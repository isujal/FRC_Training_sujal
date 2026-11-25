package frc.robot.testbench;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * The shared part of every test mechanism (rollers AND position mechanisms):
 *
 * <ul>
 *   <li>creates the leader + follower TalonFXs from a {@link MechanismConfig}</li>
 *   <li>live tunables for gains / current limits / max volts (re-sent to the motors when changed)</li>
 *   <li>SAFETY: stall cut-off, temperature cut-off, E-stop. A tripped mechanism stays stopped
 *       until you press B (or "Clear Fault" in Elastic).</li>
 *   <li>telemetry for every motor under TestBench/&lt;name&gt;/...</li>
 * </ul>
 *
 * You never use this class directly; you use {@link RollerMechanism} or {@link PositionMechanism}.
 */
public abstract class TestMechanism extends SubsystemBase {

    protected final MechanismConfig cfg;
    protected final String dash;   // telemetry prefix: "TestBench/<name>/"
    protected final String tune;   // tunable prefix:   "<name>/"

    protected final TalonFX leader;
    private final List<TalonFX> allMotors = new ArrayList<>();

    // Live-tunable motor settings. Changing any of these re-sends the motor config.
    protected final TunableNumber maxVolts, statorLimitA, supplyLimitA;
    protected final TunableNumber kP, kI, kD, kS, kV, kA;
    private final List<TunableNumber> configTunables = new ArrayList<>();

    // Live-tunable safety settings.
    private final TunableNumber stallCurrentA, stallTimeS, tempCutoffC;

    private final List<MotorSignals> signals = new ArrayList<>();
    private final BaseStatusSignal[] allSignals;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    private String fault = "";          // "" = no fault
    private boolean brakeMode;
    private double stallStartTime = Double.NaN;
    private boolean stallCheckPaused = false;
    private boolean wasEnabled = false;

    /** Test buttons shown in Elastic, filled in by the subclass. */
    protected final Map<String, Command> tests = new LinkedHashMap<>();

    protected TestMechanism(MechanismConfig cfg) {
        this.cfg = cfg;
        this.dash = "TestBench/" + cfg.name + "/";
        this.tune = cfg.name + "/";
        this.brakeMode = cfg.brakeMode;

        CANBus bus = new CANBus(cfg.canBus);
        leader = new TalonFX(cfg.leaderId, bus);
        allMotors.add(leader);
        for (MechanismConfig.FollowerSpec f : cfg.followers) {
            allMotors.add(new TalonFX(f.canId(), bus));
        }
        for (TalonFX m : allMotors) {
            signals.add(new MotorSignals(m));
        }
        List<BaseStatusSignal> list = new ArrayList<>();
        for (MotorSignals s : signals) list.addAll(s.asList());
        allSignals = list.toArray(new BaseStatusSignal[0]);

        maxVolts     = configTunable("MaxVolts", cfg.maxVolts);
        statorLimitA = configTunable("StatorLimitA", cfg.statorLimitA);
        supplyLimitA = configTunable("SupplyLimitA", cfg.supplyLimitA);
        kP = configTunable("Gains/kP", cfg.kP);
        kI = configTunable("Gains/kI", cfg.kI);
        kD = configTunable("Gains/kD", cfg.kD);
        kS = configTunable("Gains/kS", cfg.kS);
        kV = configTunable("Gains/kV", cfg.kVOrEstimate());
        kA = configTunable("Gains/kA", cfg.kA);

        stallCurrentA = new TunableNumber(tune + "Safety/StallCurrentA", 60.0);
        stallTimeS    = new TunableNumber(tune + "Safety/StallTimeS", 1.0);
        tempCutoffC   = new TunableNumber(tune + "Safety/TempCutoffC", 75.0);
    }

    /** Make a tunable that, when changed, re-sends the motor configuration. */
    protected final TunableNumber configTunable(String key, double codeDefault) {
        TunableNumber t = new TunableNumber(tune + key, codeDefault);
        configTunables.add(t);
        return t;
    }

    /** Subclasses call this LAST in their constructor (after creating their own tunables). */
    protected final void finishSetup() {
        applyMotorConfig();
        setDefaultCommand(run(this::rest).withName("Idle"));
    }

    // ── Motor configuration ────────────────────────────────────────────────────

    /** Builds the full TalonFX config from the current tunable values and sends it to every motor. */
    protected final void applyMotorConfig() {
        TalonFXConfiguration c = new TalonFXConfiguration();
        c.MotorOutput.Inverted = cfg.inverted ? InvertedValue.Clockwise_Positive
                                              : InvertedValue.CounterClockwise_Positive;
        c.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        // This one line is what makes every speed/position "mechanism units" instead of motor units.
        c.Feedback.SensorToMechanismRatio = cfg.gearRatio;

        c.Voltage.PeakForwardVoltage = Math.abs(maxVolts.get());
        c.Voltage.PeakReverseVoltage = -Math.abs(maxVolts.get());

        c.CurrentLimits.StatorCurrentLimit = statorLimitA.get();
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        c.CurrentLimits.SupplyCurrentLimit = supplyLimitA.get();
        c.CurrentLimits.SupplyCurrentLimitEnable = true;

        c.Slot0.kP = kP.get();
        c.Slot0.kI = kI.get();
        c.Slot0.kD = kD.get();
        c.Slot0.kS = kS.get();
        c.Slot0.kV = kV.get();
        c.Slot0.kA = kA.get();

        addToMotorConfig(c); // roller / position specific parts

        for (TalonFX m : allMotors) {
            m.getConfigurator().apply(c, 0.05);
        }
        for (int i = 0; i < cfg.followers.size(); i++) {
            MechanismConfig.FollowerSpec f = cfg.followers.get(i);
            allMotors.get(i + 1).setControl(new Follower(cfg.leaderId,
                    f.opposed() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }
    }

    /** Roller / position add their own settings (Motion Magic, soft limits, gravity...). */
    protected abstract void addToMotorConfig(TalonFXConfiguration c);

    /** Brake or coast for every motor of this mechanism (works while disabled). */
    public void setBrakeMode(boolean brake) {
        brakeMode = brake;
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        for (TalonFX m : allMotors) m.setNeutralMode(mode);
    }

    // ── Driving the motor (everything goes through send(), which respects faults) ─

    /** Send a control request to the leader, unless the mechanism is faulted. */
    protected final void send(ControlRequest request) {
        leader.setControl(isFaulted() ? neutralRequest : request);
    }

    /** Open-loop volts (clamped to MaxVolts). */
    public void runVolts(double volts) {
        double max = Math.abs(maxVolts.get());
        send(voltageRequest.withOutput(MathUtil.clamp(volts, -max, max)));
    }

    /** Motor output off (coast or brake depending on mode). */
    public void stop() {
        leader.setControl(neutralRequest);
    }

    /** What the mechanism does when nothing is commanding it. Roller = stop, position = hold. */
    public abstract void rest();

    // ── Safety ─────────────────────────────────────────────────────────────────

    /** Latch a fault: stop now, cancel whatever was running, refuse to run until cleared. */
    public void trip(String reason) {
        if (isFaulted()) return;
        fault = reason;
        Command running = getCurrentCommand();
        if (running != null) running.cancel();
        stop();
        DriverStation.reportWarning("[TestBench] " + cfg.name + " STOPPED: " + reason, false);
    }

    public void clearFault() {
        fault = "";
        stallStartTime = Double.NaN;
    }

    public boolean isFaulted() { return !fault.isEmpty(); }
    public String getFault()   { return fault; }

    /** Tests that push into a hard stop on purpose (homing) pause the stall check. */
    public void setStallCheckPaused(boolean paused) {
        stallCheckPaused = paused;
        stallStartTime = Double.NaN;
    }

    private void checkSafety() {
        double now = Timer.getFPGATimestamp();

        // Stall: lots of current but (almost) not moving, for longer than StallTimeS.
        boolean stalling = getStatorCurrentA() > stallCurrentA.get() && Math.abs(getMotorRPM()) < 60.0;
        if (stalling && !stallCheckPaused) {
            if (Double.isNaN(stallStartTime)) stallStartTime = now;
            if (now - stallStartTime > stallTimeS.get()) {
                trip(String.format("STALL (%.0f A for %.1f s)", getStatorCurrentA(), stallTimeS.get()));
            }
        } else {
            stallStartTime = Double.NaN;
        }

        // Temperature: hottest motor of this mechanism.
        if (getMaxTempC() > tempCutoffC.get()) {
            trip(String.format("OVER TEMP (%.0f C)", getMaxTempC()));
        }
    }

    // ── Readings (all in mechanism units) ──────────────────────────────────────

    /** Leader speed in mechanism RPM. */
    public double getVelocityRPM()   { return signals.get(0).velocity.getValueAsDouble() * 60.0; }
    /** Leader speed in motor RPM (compare with the motor free speed, e.g. 6000 for a Kraken). */
    public double getMotorRPM()      { return getVelocityRPM() * cfg.gearRatio; }
    /** Leader position in mechanism rotations. */
    public double getPositionRot()   { return signals.get(0).position.getValueAsDouble(); }
    public double getStatorCurrentA(){ return signals.get(0).stator.getValueAsDouble(); }
    public double getSupplyCurrentA(){ return signals.get(0).supply.getValueAsDouble(); }
    public double getAppliedVolts()  { return signals.get(0).volts.getValueAsDouble(); }

    /** Sum of stator current of all motors. */
    public double getTotalStatorA() {
        double sum = 0;
        for (MotorSignals s : signals) sum += s.stator.getValueAsDouble();
        return sum;
    }

    public double getMaxTempC() {
        double max = 0;
        for (MotorSignals s : signals) max = Math.max(max, s.temp.getValueAsDouble());
        return max;
    }

    public String getName()              { return cfg.name; }
    public MechanismConfig getConfig()   { return cfg; }
    public double getMaxVolts()          { return Math.abs(maxVolts.get()); }
    public Map<String, Command> getTests() { return tests; }

    // ── Controller hooks (TestBenchContainer maps buttons to these) ─────────────

    /** A button. */
    public abstract Command primaryCommand();
    /** Y button. */
    public abstract Command secondaryCommand();
    /** X button: the most common test for this kind of mechanism. */
    public abstract Command quickTestCommand();
    /** POV up (+1) / down (-1): roller = change target RPM, position = jog. */
    public abstract Command nudgeCommand(int direction);
    /** Triggers: open-loop control. input is -1..1. */
    public abstract Command manualCommand(DoubleSupplier input);
    /** Start (held 1 s): position = "the mechanism is at its start position now". Roller: nothing. */
    public void zeroHere() {}

    // ── Every loop ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(allSignals);

        // Re-send the motor config if any gain / limit was edited in Elastic.
        boolean changed = false;
        for (TunableNumber t : configTunables) {
            changed |= t.hasChanged(); // |= (not ||) so every tunable is checked each loop
        }
        if (changed) {
            applyMotorConfig();
            System.out.println("[TestBench] " + cfg.name + ": new gains/limits sent to motors");
        }

        boolean enabled = DriverStation.isEnabled();
        if (enabled && !wasEnabled) onEnabled();
        wasEnabled = enabled;
        if (enabled) checkSafety();

        publishCommonTelemetry();
        publishTelemetry();
    }

    /** Called once each time the robot is enabled. */
    protected void onEnabled() {}

    /** Subclass-specific telemetry. */
    protected abstract void publishTelemetry();

    private void publishCommonTelemetry() {
        Command c = getCurrentCommand();
        SmartDashboard.putString(dash + "Running", c == null ? "none" : c.getName());
        SmartDashboard.putBoolean(dash + "OK (no fault)", !isFaulted());
        SmartDashboard.putString(dash + "Fault", isFaulted() ? fault : "none");
        SmartDashboard.putBoolean(dash + "Brake Mode", brakeMode);
        SmartDashboard.putNumber(dash + "Speed RPM", getVelocityRPM());
        SmartDashboard.putNumber(dash + "Motor RPM", getMotorRPM());
        SmartDashboard.putNumber(dash + "Applied Volts", getAppliedVolts());
        SmartDashboard.putNumber(dash + "Stator A", getStatorCurrentA());
        SmartDashboard.putNumber(dash + "Supply A", getSupplyCurrentA());
        SmartDashboard.putNumber(dash + "Total Stator A", getTotalStatorA());
        SmartDashboard.putNumber(dash + "Max Temp C", getMaxTempC());

        // Every motor separately: shows at once if a follower is not following / is fighting.
        for (int i = 0; i < signals.size(); i++) {
            MotorSignals s = signals.get(i);
            String p = dash + "Motors/" + (i == 0 ? "Leader " : "Follower ") + allMotors.get(i).getDeviceID() + "/";
            SmartDashboard.putNumber(p + "Speed RPM", s.velocity.getValueAsDouble() * 60.0);
            SmartDashboard.putNumber(p + "Stator A", s.stator.getValueAsDouble());
            SmartDashboard.putNumber(p + "Temp C", s.temp.getValueAsDouble());
        }
    }

    /** The status signals we read from one motor. */
    private static final class MotorSignals {
        final BaseStatusSignal position, velocity, stator, supply, temp, volts;

        MotorSignals(TalonFX m) {
            position = m.getPosition(false);
            velocity = m.getVelocity(false);
            stator = m.getStatorCurrent(false);
            supply = m.getSupplyCurrent(false);
            temp = m.getDeviceTemp(false);
            volts = m.getMotorVoltage(false);
        }

        List<BaseStatusSignal> asList() {
            return List.of(position, velocity, stator, supply, temp, volts);
        }
    }
}
