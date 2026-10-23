package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class FollowAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private final PIDController rotationPID = new PIDController(0.035, 0.0, 0.002);
    private final PIDController rangePID = new PIDController(0.6, 0.0, 0.03);

    private static final double TARGET_AREA = 0.05; // tune: ta value at your desired following distance
    private static final double MAX_ROTATION_SPEED = 1.5; // rad/s
    private static final double MAX_FORWARD_SPEED = 3;  // m/s

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public FollowAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
        rotationPID.setTolerance(1.0);
        rangePID.setTolerance(0.1);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double rotationOutput = clamp(-rotationPID.calculate(vision.getTx(), 0.0),
                    -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            double forwardOutput = clamp(rangePID.calculate(vision.getTa(), TARGET_AREA),
                    -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);

            drivetrain.setControl(driveRequest
                    .withVelocityX(forwardOutput)
                    .withVelocityY(0.0)
                    .withRotationalRate(rotationOutput));
        } else {
            drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false; // keeps running while bumper held
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}