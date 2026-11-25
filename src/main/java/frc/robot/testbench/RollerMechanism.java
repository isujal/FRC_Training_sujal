package frc.robot.testbench;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

/**
 * Preset for anything that SPINS: intake rollers, feeder, indexer, conveyor, gripper wheels, shooter.
 *
 * <pre>
 *  Tester controller (port 0)   What it does
 *  A                            run forward at TargetRPM (closed loop) until B
 *  Y                            run reverse at TargetRPM until B
 *  POV up / down                TargetRPM +/- RPMStep  (live, even while running)
 *  Right / left trigger         open-loop volts forward / reverse (trigger x MaxVolts)
 *  X                            test "Run Until Current Spike"
 * </pre>
 *
 * All speeds are mechanism RPM (see {@link MechanismConfig}).
 */
public class RollerMechanism extends TestMechanism {

    private final TunableNumber targetRPM, rpmStep, atSpeedTolRPM;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private double commandedRPM = 0.0;

    public RollerMechanism(MechanismConfig cfg) {
        super(cfg);
        targetRPM     = new TunableNumber(tune + "TargetRPM", cfg.targetRPM);
        rpmStep       = new TunableNumber(tune + "RPMStep", 100.0);
        atSpeedTolRPM = new TunableNumber(tune + "AtSpeedTolRPM", Math.max(50.0, 0.03 * cfg.targetRPM));
        finishSetup();

        RollerTests t = new RollerTests(this);
        tests.put("Run Until Current Spike", t.runUntilCurrent());
        tests.put("Endurance Soak", t.soak());
        tests.put("Jam Detect + Auto Unjam", t.jam());
        tests.put("Speed Ramp + Free Speed Check", t.speedRamp());
        tests.put("Flywheel Spin-up + Recovery", t.flywheel());
    }

    @Override
    protected void addToMotorConfig(TalonFXConfiguration c) {
        // Nothing extra for a roller. (Velocity gains are already in Slot0.)
    }

    // ── Actions ────────────────────────────────────────────────────────────────

    /** Closed-loop speed, mechanism RPM. Negative = reverse. */
    public void runRPM(double rpm) {
        commandedRPM = rpm;
        send(velocityRequest.withVelocity(rpm / 60.0)); // Phoenix wants rotations per SECOND
    }

    @Override
    public void runVolts(double volts) {
        commandedRPM = 0.0;
        super.runVolts(volts);
    }

    @Override
    public void rest() {
        commandedRPM = 0.0;
        stop();
    }

    public double getTargetRPM()     { return targetRPM.get(); }
    public double getAtSpeedTolRPM() { return atSpeedTolRPM.get(); }

    public boolean isAtSpeed() {
        return commandedRPM != 0.0 && Math.abs(commandedRPM - getVelocityRPM()) < atSpeedTolRPM.get();
    }

    // ── Controller hooks ───────────────────────────────────────────────────────

    @Override
    public Command primaryCommand() {
        // Reads TargetRPM every loop, so editing it in Elastic changes the speed immediately.
        return run(() -> runRPM(targetRPM.get())).withName("Run Forward");
    }

    @Override
    public Command secondaryCommand() {
        return run(() -> runRPM(-targetRPM.get())).withName("Run Reverse");
    }

    @Override
    public Command quickTestCommand() {
        return tests.get("Run Until Current Spike");
    }

    @Override
    public Command nudgeCommand(int direction) {
        // No requirement on purpose: changing the speed must NOT stop the roller that is running.
        return Commands.runOnce(() -> targetRPM.set(targetRPM.get() + direction * rpmStep.get()))
                .withName(direction > 0 ? "RPM +" : "RPM -");
    }

    @Override
    public Command manualCommand(DoubleSupplier input) {
        return run(() -> runVolts(input.getAsDouble() * getMaxVolts())).withName("Manual Volts");
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────

    @Override
    protected void publishTelemetry() {
        SmartDashboard.putNumber(dash + "Target RPM (tunable)", targetRPM.get());
        SmartDashboard.putNumber(dash + "Commanded RPM", commandedRPM);
        SmartDashboard.putNumber(dash + "Error RPM", commandedRPM == 0.0 ? 0.0 : commandedRPM - getVelocityRPM());
        SmartDashboard.putBoolean(dash + "At Speed", isAtSpeed());
        SmartDashboard.putNumber(dash + "Expected Free Speed RPM", cfg.mechanismFreeSpeedRPM());
    }
}
