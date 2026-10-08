// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
// import frc.robot.commands.Basic_Command;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Basic_Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

public class RobotContainer {
    // private final Basic_Subsystem m_subsystem = new Basic_Subsystem();
    public boolean motor2Running = false;
    private double speed = 0.2;
    private final Intake intake = new Intake();
    private final Indexer indexer = new Indexer();
    private final Shooter shooter = new Shooter();

    private double MaxSpeed = speed * 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                              // top speed1
    private double MaxAngularRate = speed * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                              // second max angular
                                                                                              // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private boolean slowMode = false;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;
    private final Field2d field = new Field2d();

    public RobotContainer() {
        registerNamedCommands(); // 1) register names first
        autoChooser = AutoBuilder.buildAutoChooser(); // testing only: this auto is the default
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void registerNamedCommands() {

        NamedCommands.registerCommand("IntakeOn", traced("IntakeOn", new InstantCommand(() -> intake.setIntakeRollerRPM(80)).alongWith(new InstantCommand(()-> intake.setIntakePosition(135)))) 
        );
        NamedCommands.registerCommand("Intake Off", traced("Intake Off", new InstantCommand(() -> intake.stopMotor())));
        NamedCommands.registerCommand("FeederOn", traced("FeederOn", new InstantCommand(() -> indexer.setIndexerRPM(6000)).alongWith(new InstantCommand(()-> indexer.setFeederRPM(4200)))) 
        );
        NamedCommands.registerCommand("Shooter Command", traced("Shooter Command", new ShooterCommand(shooter)));

    }

    private static Command traced(String name, Command c) {
        return c.beforeStarting(() -> {
            System.out.println("[AUTO] start " + name);
            SmartDashboard.putBoolean("Auto/" + name + " running", true);
        })
                .finallyDo(interrupted -> {
                    System.out.println("[AUTO] end   " + name + (interrupted ? " (cancelled)" : ""));
                    SmartDashboard.putBoolean("Auto/" + name + " running", false);
                });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected(); // replaces the template drive-forward
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    Translation2d v = driverVelocity();
                    double scale = slowMode ? DriveConstants.kSlowModeScale : 1.0;
                    return drive
                            .withVelocityX(xLimiter.calculate(v.getX()))
                            .withVelocityY(yLimiter.calculate(v.getY()))
                            .withRotationalRate(shapeAxis(-joystick.getRightX()) * MaxAngularRate * scale);
                }).beforeStarting(() -> {
                    // Start the limiters at the current stick value so the robot doesn't
                    // lurch when this command resumes after brake, heading lock, etc.
                    Translation2d v = driverVelocity();
                    xLimiter.reset(v.getX());
                    yLimiter.reset(v.getY());
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.leftTrigger().whileTrue(
                Commands.startEnd(() -> slowMode = true, () -> slowMode = false));
        // Reset the field-centric heading on left bumper press.

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povLeft().toggleOnTrue(new IntakeCommand(intake));
        // joystick.povRight().onTrue(new InstantCommand(() -> shooter.decrementHood(0.8), shooter));
        joystick.povUp().onTrue(new InstantCommand(() -> intake.incrementPosition(5), intake));
        joystick.povDown().onTrue(new InstantCommand(() -> intake.decrementPosition(5), intake));
        // toggle logic
        // joystick.rightBumper().onTrue(new InstantCommand(() -> {
        // motor2Running = !motor2Running;
        // if (motor2Running) {
        // intake.IntakeRollerRPM(60);
        // }
        // else {
        // intake.stopMotor();;
        // }
        // }, intake));

        // joystick.leftBumper().toggleOnTrue(new IntakeCommand(intake));
        joystick.rightBumper().toggleOnTrue(new ShooterCommand(shooter));
        joystick.y().toggleOnTrue(new FeederCommand(indexer, 6000, 4200));

        // ── Motor 1 (MotionMagic Position) ────────────────────────────────────

        // joystick.a().onTrue(new InstantCommand(
        // () -> m_subsystem.setMotor1Position(10.0), m_subsystem));

        // joystick.b().onTrue(new InstantCommand(
        // () -> m_subsystem.setMotor1Position(0.0), m_subsystem));

        // joystick.povUp().onTrue(new InstantCommand(
        // () -> m_subsystem.incrementPosition(5.0), m_subsystem));

        // joystick.povDown().onTrue(new InstantCommand(
        // () -> m_subsystem.decrementPosition(5.0), m_subsystem));

        // joystick.back().onTrue(new InstantCommand(
        // () -> m_subsystem.zeroMotor1(), m_subsystem));

        // joystick.rightBumper().whileTrue(new Basic_Command(m_subsystem, 60.0));

        // joystick.leftBumper().whileTrue(new Basic_Command(m_subsystem, -20.0));

        // joystick.back().onTrue(new Basic_Command(m_subsystem, 60.0));

        // joystick.y().onTrue(new InstantCommand(
        // () -> m_subsystem.stopMotor2(), m_subsystem));

        // // toggle logic
        // joystick.x().onTrue(new InstantCommand(() -> {
        // motor2Running = !motor2Running;
        // if (motor2Running) {
        // m_subsystem.setMotor2Velocity(60.0);
        // } else {
        // m_subsystem.stopMotor2();
        // }
        // }, m_subsystem));
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
        PathPlannerLogging.setLogTargetPoseCallback(pose -> field.getObject("target").setPose(pose));

    }

    /**
     * Stick (-1..1 each) to a vector of magnitude 0..1: circular deadband + squared
     * curve.
     */
    private static Translation2d shapeStick(double x, double y) {
        double db = DriveConstants.kStickDeadband;
        double mag = Math.hypot(x, y);
        if (mag < db) {
            return new Translation2d();
        }
        double scaled = MathUtil.clamp((mag - db) / (1.0 - db), 0.0, 1.0);
        scaled = scaled * scaled;
        return new Translation2d(x / mag * scaled, y / mag * scaled);
    }

    /** Single axis (rotation): deadband + squared curve, keeping the sign. */
    private static double shapeAxis(double v) {
        v = MathUtil.applyDeadband(v, DriveConstants.kStickDeadband);
        return Math.copySign(v * v, v);
    }

    /** Driver's requested field velocity in m/s, before the slew limiter. */
    private Translation2d driverVelocity() {
        double scale = slowMode ? DriveConstants.kSlowModeScale : 1.0;
        // WPILib: stick forward is negative Y, stick left is negative X
        return shapeStick(-joystick.getLeftY(), -joystick.getLeftX()).times(MaxSpeed * scale);
    }

    /** Call every loop so the dashboard shows the live robot pose. */
    public void updateField() {
        field.setRobotPose(drivetrain.getState().Pose);
    }

    // public Command getAutonomousCommand() {
    // // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    // // Reset our field centric heading to match the robot
    // // facing away from our alliance station wall (0 deg).
    // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    // // Then slowly drive forward (away from us) for 5 seconds.
    // drivetrain.applyRequest(() ->
    // drive.withVelocityX(0.5)
    // .withVelocityY(0)
    // .withRotationalRate(0)
    // )
    // .withTimeout(5.0),
    // // Finally idle for the rest of auton
    // drivetrain.applyRequest(() -> idle)
    // );
    // }
}
