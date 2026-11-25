package frc.robot.testbench;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.function.Function;

/**
 * The robot when Constants.kTestBenchMode = true. Replaces RobotContainer completely
 * (so the two never fight over the same CAN IDs).
 *
 * <pre>
 *  TESTER controller = port 1. It controls ONE mechanism at a time, shown in Elastic as
 *  "TestBench/Controlling". Change it with the "TestBench/Mechanism" dropdown OR the bumpers.
 *
 *  RB / LB ...... next / previous mechanism (also moves the Elastic dropdown)
 *  A ............ roller: run forward at TargetRPM   | position: go to Preset A
 *  Y ............ roller: run reverse                | position: go to Preset B
 *  B ............ STOP this mechanism (position: hold here) + clear its fault
 *  X ............ quick test (roller: Run Until Current Spike | position: Step Response)
 *  POV up/down .. roller: TargetRPM +/- RPMStep      | position: jog +/- JogStep
 *  Triggers ..... open-loop volts (right = +, left = -)
 *  Start (hold 1 s) ... position: re-zero encoder at start position
 *  Back ......... E-STOP: every mechanism stops and latches a fault
 *
 *  DRIVER controller = port 0 (only if kIncludeDrive): slow field-centric drive, LB = reset heading.
 * </pre>
 */
public class TestBenchContainer {

    /** Drive the robot around while testing? Set false on a bench / when the drivebase is apart. */
    private static final boolean kIncludeDrive = true;
    private static final double kDriveSpeedScale = 0.3; // 30% of top speed while testing

    private final List<TestMechanism> mechanisms = MechanismPresets.create();
    private final SendableChooser<TestMechanism> chooser = new SendableChooser<>();
    private final CommandXboxController tester = new CommandXboxController(1);
    private final CommandXboxController driver = new CommandXboxController(0);

    public TestBenchContainer() {
        // Record everything (all dashboard values + DS/joystick data) to a .wpilog for AdvantageScope.
        // Saved on a USB stick if one is plugged into the roboRIO, else /home/lvuser/logs.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        setUpDashboard();
        bindTesterController();
        if (kIncludeDrive) setUpDrive();

        DataLogManager.log("[TestBench] Ready with " + mechanisms.size() + " mechanism(s).");
    }

    // ── Elastic ────────────────────────────────────────────────────────────────

    private void setUpDashboard() {
        for (int i = 0; i < mechanisms.size(); i++) {
            TestMechanism m = mechanisms.get(i);
            if (i == 0) chooser.setDefaultOption(m.getName(), m);
            else chooser.addOption(m.getName(), m);

            String p = "TestBench/" + m.getName() + "/";
            // One button per test. Click = start, click again = cancel.
            m.getTests().forEach((name, cmd) -> SmartDashboard.putData(p + "Tests/" + name, cmd));
            SmartDashboard.putData(p + "Stop + Clear Fault", stopCommand(m));
            SmartDashboard.putData(p + "Brake Mode", Commands.runOnce(() -> m.setBrakeMode(true))
                    .ignoringDisable(true).withName("Brake"));
            SmartDashboard.putData(p + "Coast Mode", Commands.runOnce(() -> m.setBrakeMode(false))
                    .ignoringDisable(true).withName("Coast"));
        }
        SmartDashboard.putData("TestBench/Mechanism", chooser);
        SmartDashboard.putData("TestBench/E-STOP ALL", eStopCommand());
        SmartDashboard.putData("TestBench/Clear All Faults",
                Commands.runOnce(() -> mechanisms.forEach(TestMechanism::clearFault))
                        .ignoringDisable(true).withName("Clear All Faults"));
        SmartDashboard.putData("TestBench/Print All Tunables",
                Commands.runOnce(TunableNumber::printAll).ignoringDisable(true).withName("Print"));
        SmartDashboard.putData("TestBench/Reset Tunables To Code",
                Commands.runOnce(TunableNumber::resetAllToCode).ignoringDisable(true).withName("Reset"));
    }

    /** Writing here is exactly what Elastic does when you pick from the dropdown. */
    private final StringPublisher dropdownSelection = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/TestBench/Mechanism/selected").publish();

    /** RB = +1 (next), LB = -1 (previous). Wraps around at the ends of the list. */
    private void selectNext(int direction) {
        int i = mechanisms.indexOf(selected());
        TestMechanism next = mechanisms.get(Math.floorMod(i + direction, mechanisms.size()));
        dropdownSelection.set(next.getName()); // the dropdown is the one source of truth
        System.out.println("[TestBench] Controller now controls: " + next.getName());
    }

    private TestMechanism selected() {
        TestMechanism m = chooser.getSelected();
        return m != null ? m : mechanisms.get(0);
    }

    private Command stopCommand(TestMechanism m) {
        // runOnce(m) REQUIRES m, so it also cancels whatever test/command was running on it.
        return m.runOnce(() -> {
            m.clearFault();
            m.rest();
        }).withName("Stop");
    }

    private Command eStopCommand() {
        return Commands.runOnce(() -> mechanisms.forEach(m -> m.trip("E-STOP pressed")))
                .ignoringDisable(true).withName("E-STOP");
    }

    // ── Tester controller ──────────────────────────────────────────────────────

    private void bindTesterController() {
        onPress(tester.a(), TestMechanism::primaryCommand);
        onPress(tester.y(), TestMechanism::secondaryCommand);
        onPress(tester.x(), TestMechanism::quickTestCommand);
        onPress(tester.b(), this::stopCommand);
        onPress(tester.povUp(),   m -> m.nudgeCommand(+1));
        onPress(tester.povDown(), m -> m.nudgeCommand(-1));
        onPress(tester.start().debounce(1.0), m -> m.runOnce(m::zeroHere).withName("Zero Here"));
        tester.back().onTrue(eStopCommand());
        tester.rightBumper().onTrue(Commands.runOnce(() -> selectNext(+1)).ignoringDisable(true));
        tester.leftBumper().onTrue(Commands.runOnce(() -> selectNext(-1)).ignoringDisable(true));

        // Triggers: runs while either trigger is pressed, stops when both are released.
        Trigger anyTrigger = tester.rightTrigger(0.05).or(tester.leftTrigger(0.05));
        onPress(anyTrigger, m -> m.manualCommand(this::triggerInput)
                .until(() -> Math.abs(triggerInput()) < 0.05)
                .withName("Manual Volts"));
    }

    private double triggerInput() {
        return tester.getRightTriggerAxis() - tester.getLeftTriggerAxis();
    }

    /** When the button is pressed, build the command for the CURRENTLY selected mechanism and run it. */
    private void onPress(Trigger button, Function<TestMechanism, Command> commandFor) {
        button.onTrue(Commands.runOnce(() ->
                CommandScheduler.getInstance().schedule(commandFor.apply(selected()))));
    }

    // ── Optional drive ─────────────────────────────────────────────────────────

    private void setUpDrive() {
        CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        double maxSpeed = kDriveSpeedScale * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double maxTurn = kDriveSpeedScale * RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxTurn * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
                .withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), 0.1) * maxSpeed)
                .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * maxSpeed)
                .withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * maxTurn)));

        SwerveRequest.Idle idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    // ── Every loop (called from Robot.robotPeriodic) ───────────────────────────

    public void periodic() {
        TestMechanism m = selected();
        // The mechanism the tester controller drives right now, e.g. "Feeder (4/6)".
        SmartDashboard.putString("TestBench/Controlling",
                m.getName() + "  (" + (mechanisms.indexOf(m) + 1) + "/" + mechanisms.size() + ")");
        SmartDashboard.putString("TestBench/Running Now", runningNow());
        SmartDashboard.putString("TestBench/Controlling Last Result",
                SmartDashboard.getString("TestBench/" + m.getName() + "/Last Result", "-"));
        SmartDashboard.putNumber("TestBench/Battery V", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("TestBench/Tunables Changed From Code", TunableNumber.countChangedFromCode());

        // Every 0.5 s (no need every loop): the list of values that differ from code.
        if (++reportLoops >= 25) {
            reportLoops = 0;
            SmartDashboard.putString("TestBench/Tunables Report", TunableNumber.changedReport());
        }
    }

    /** Every mechanism that is doing something (not idle), e.g. "Intake Rollers: Run Forward | Feeder: Endurance Soak". */
    private String runningNow() {
        StringBuilder sb = new StringBuilder();
        for (TestMechanism m : mechanisms) {
            Command c = m.getCurrentCommand();
            String what = m.isFaulted() ? "FAULT - " + m.getFault()
                    : (c == null || c.getName().equals("Idle")) ? null : c.getName();
            if (what != null) {
                if (sb.length() > 0) sb.append("  |  ");
                sb.append(m.getName()).append(": ").append(what);
            }
        }
        return sb.length() == 0 ? "nothing running" : sb.toString();
    }

    private int reportLoops = 25; // so the report is published on the very first loop
}
