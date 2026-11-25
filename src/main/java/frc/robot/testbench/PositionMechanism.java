package frc.robot.testbench;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

/**
 * Preset for anything that MOVES TO A POSITION: arm, elbow, shoulder, wrist, hood, pivot, turret,
 * gripper pivot, elevator / telescope (use {@link MechanismConfig#units} for inches or meters).
 *
 * <pre>
 *  Tester controller (port 0)   What it does
 *  A / Y                        go to Preset A / Preset B (follows the preset live while held active)
 *  POV up / down                jog the target by +/- JogStep
 *  Right / left trigger         open-loop volts (trigger x ManualVolts). Release = hold where it is
 *  X                            test "Step Response"
 *  Start (hold 1 s)             "the mechanism is at its start position right now" (re-zero)
 *  B                            hold where it is now + clear fault
 * </pre>
 *
 * Positions are in cfg.unitName (degrees by default), measured at the mechanism, not the motor.
 * When nothing is commanding it, it HOLDS its last target (it does not go limp).
 */
public class PositionMechanism extends TestMechanism {

    private final TunableNumber kG, cruise, accel, jerk;
    private final TunableNumber presetA, presetB, jogStep, manualVolts, toleranceUnits;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    private double target;                 // in units (deg / in / m)
    private boolean softLimitsEnabled = true;

    public PositionMechanism(MechanismConfig cfg) {
        super(cfg);
        String u = cfg.unitName;
        kG     = configTunable("Gains/kG", cfg.kG);
        cruise = configTunable("MotionMagic/CruiseVel (" + u + " per s)", cfg.cruiseVelocity);
        accel  = configTunable("MotionMagic/Accel (" + u + " per s^2)", cfg.acceleration);
        jerk   = configTunable("MotionMagic/Jerk (" + u + " per s^3, 0 = off)", cfg.jerk);

        double range = cfg.maxPosition - cfg.minPosition;
        presetA        = new TunableNumber(tune + "PresetA (" + u + ")",
                Double.isNaN(cfg.presetA) ? cfg.minPosition + 0.25 * range : cfg.presetA);
        presetB        = new TunableNumber(tune + "PresetB (" + u + ")",
                Double.isNaN(cfg.presetB) ? cfg.minPosition + 0.75 * range : cfg.presetB);
        jogStep        = new TunableNumber(tune + "JogStep (" + u + ")", 0.05 * range);
        manualVolts    = new TunableNumber(tune + "ManualVolts", 3.0);
        toleranceUnits = new TunableNumber(tune + "Tolerance (" + u + ")", 0.01 * range);

        target = cfg.startPosition;
        finishSetup(); // sends the config (gear ratio!) first...
        // ...then: the robot assumes the mechanism is sitting at startPosition when it turns on.
        leader.setPosition(toRot(cfg.startPosition));

        PositionTests t = new PositionTests(this);
        tests.put("Home To Hard Stop", t.home());
        tests.put("Range Sweep", t.sweep());
        tests.put("Step Response", t.stepResponse());
        tests.put("Repeatability", t.repeatability());
        tests.put("Hold Test", t.hold());
        tests.put("Find kS + kG", t.findKsKg());
    }

    @Override
    protected void addToMotorConfig(TalonFXConfiguration c) {
        c.Slot0.kG = kG.get();
        c.Slot0.GravityType = cfg.gravity == MechanismConfig.Gravity.ARM
                ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
        if (cfg.gravity == MechanismConfig.Gravity.NONE) c.Slot0.kG = 0.0;

        // Motion Magic is in mechanism rotations / s because of SensorToMechanismRatio.
        c.MotionMagic.MotionMagicCruiseVelocity = toRot(cruise.get());
        c.MotionMagic.MotionMagicAcceleration = toRot(accel.get());
        c.MotionMagic.MotionMagicJerk = toRot(Math.max(0.0, jerk.get()));

        c.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitsEnabled;
        c.SoftwareLimitSwitch.ForwardSoftLimitThreshold = toRot(cfg.maxPosition);
        c.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitsEnabled;
        c.SoftwareLimitSwitch.ReverseSoftLimitThreshold = toRot(cfg.minPosition);
    }

    // ── Units ──────────────────────────────────────────────────────────────────
    private double toRot(double units)  { return units / cfg.unitsPerRotation; }
    private double toUnits(double rot)  { return rot * cfg.unitsPerRotation; }

    // ── Actions ────────────────────────────────────────────────────────────────

    /** Motion Magic to a position (clamped to the range in the config). */
    public void setTarget(double units) {
        target = MathUtil.clamp(units, cfg.minPosition, cfg.maxPosition);
        send(motionMagicRequest.withPosition(toRot(target)));
    }

    /** Follow a slowly moving setpoint directly (no Motion Magic). Used by the sweep test. */
    public void setTrackingSetpoint(double units, double unitsPerSec) {
        target = MathUtil.clamp(units, cfg.minPosition, cfg.maxPosition);
        send(positionRequest.withPosition(toRot(target)).withVelocity(toRot(unitsPerSec)));
    }

    /** Open-loop volts. The target follows the real position, so when you let go it holds THERE. */
    @Override
    public void runVolts(double volts) {
        target = MathUtil.clamp(getPosition(), cfg.minPosition, cfg.maxPosition);
        super.runVolts(volts);
    }

    /** Make "where it is right now" the target, so it holds still. */
    public void holdCurrentPosition() {
        setTarget(getPosition());
    }

    @Override
    public void rest() {
        // Re-sends the same target every loop: the arm holds instead of dropping.
        send(motionMagicRequest.withPosition(toRot(target)));
    }

    @Override
    protected void onEnabled() {
        // On enable, do not jump back to an old target - hold wherever the mechanism is now.
        target = MathUtil.clamp(getPosition(), cfg.minPosition, cfg.maxPosition);
    }

    @Override
    public void zeroHere() {
        leader.setPosition(toRot(cfg.startPosition));
        target = cfg.startPosition;
        System.out.println("[TestBench] " + cfg.name + " re-zeroed to " + cfg.startPosition + " " + cfg.unitName);
    }

    /** Set the encoder to a known position (homing). */
    public void setEncoderPosition(double units) {
        leader.setPosition(toRot(units));
        target = units;
    }

    /** Homing has to go past the soft limit, so it can switch them off for a moment. */
    public void setSoftLimitsEnabled(boolean enabled) {
        softLimitsEnabled = enabled;
        applyMotorConfig();
    }

    public double getPosition()        { return toUnits(getPositionRot()); }
    /** Speed in units per second. */
    public double getVelocityUnits()   { return getVelocityRPM() / 60.0 * cfg.unitsPerRotation; }
    public double getTarget()          { return target; }
    public double getError()           { return target - getPosition(); }
    public double getTolerance()       { return Math.abs(toleranceUnits.get()); }
    public boolean atTarget()          { return Math.abs(getError()) < getTolerance(); }
    public double getPresetA()         { return presetA.get(); }
    public double getPresetB()         { return presetB.get(); }

    // ── Controller hooks ───────────────────────────────────────────────────────

    @Override
    public Command primaryCommand() {
        return run(() -> setTarget(presetA.get())).withName("Go To Preset A");
    }

    @Override
    public Command secondaryCommand() {
        return run(() -> setTarget(presetB.get())).withName("Go To Preset B");
    }

    @Override
    public Command quickTestCommand() {
        return tests.get("Step Response");
    }

    @Override
    public Command nudgeCommand(int direction) {
        // Requires this mechanism on purpose: it replaces "Go To Preset", which would undo the jog.
        return runOnce(() -> setTarget(target + direction * jogStep.get()))
                .withName(direction > 0 ? "Jog +" : "Jog -");
    }

    @Override
    public Command manualCommand(DoubleSupplier input) {
        return run(() -> runVolts(input.getAsDouble() * Math.abs(manualVolts.get())))
                .finallyDo(this::holdCurrentPosition)
                .withName("Manual Volts");
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    @Override
    protected void publishTelemetry() {
        String u = " (" + cfg.unitName + ")";
        SmartDashboard.putNumber(dash + "Position" + u, getPosition());
        SmartDashboard.putNumber(dash + "Target" + u, target);
        SmartDashboard.putNumber(dash + "Error" + u, getError());
        SmartDashboard.putBoolean(dash + "At Target", atTarget());
        SmartDashboard.putNumber(dash + "Velocity" + u + " per s", getVelocityUnits());
        SmartDashboard.putBoolean(dash + "Soft Limits On", softLimitsEnabled);
    }
}
