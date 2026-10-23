package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain; // adjust import to match your project's Tuner X generated class

public class AprilTagTrackCommand extends Command {

    // ---- Limelight NetworkTable ----
    private final NetworkTable limelightTable =
            NetworkTableInstance.getDefault().getTable("limelight-centre"); // change if your LL name differs

    // ---- Drivetrain ----
    private final CommandSwerveDrivetrain drivetrain;

    // Robot-centric request: x = forward/back (m/s), y = strafe (m/s), rot = rad/s
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0);

    // ---- Simple proportional gains (EDIT THESE to tune feel) ----
    private static final double AIM_GAIN = 0.1;   // rad/s of rotation per degree of tx error
    private static final double RANGE_GAIN = 0.3; // m/s of forward speed per 1.0 unit of ta error

    // ---- Output caps (tune to your robot's top speeds) ----
    private static final double MAX_ROTATION_RAD_PER_SEC = Math.toRadians(180.0);
    private static final double MAX_FORWARD_MPS = 5;

    // Minimal target-area filter so we don't react to a barely-visible tag
    private static final double MIN_TARGET_AREA = 0.05;

    // Small deadbands so the robot doesn't jitter when it's already clos[]\e enough
    private static final double TX_DEADBAND_DEG = 1.0;
    private static final double TA_DEADBAND = 0.05;

    // Captured the moment the command starts (bumper pressed) - this is the
    // "distance" we try to hold for the rest of the command's run.
    private double targetArea = -1.0;

    public AprilTagTrackCommand(CommandSwerveDrivetrain drivetrain) {//constructor
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Make sure Limelight is in AprilTag pipeline / vision mode when this command starts.
        limelightTable.getEntry("camMode").setNumber(0); // 0 = vision processor mode

        // Not captured yet - execute() will grab it on the first valid frame
        // it sees, since the tag may not be visible in the exact instant the
        // bumper is pressed.
        targetArea = -1.0;
    }

    @Override
    public void execute() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0.0) == 1.0;

        double forwardSpeed = 0.0;
        double rotationSpeed = 0.0;

        if (hasTarget) {
            double tx = limelightTable.getEntry("tx").getDouble(0.0); // horizontal offset, deg
            double ta = limelightTable.getEntry("ta").getDouble(0.0); // target area, 0-100

            if (ta >= MIN_TARGET_AREA) {

                // Lock in the distance to hold as soon as we get a real reading.
                if (targetArea < 0.0) {
                    targetArea = ta;
                }

                // --- Rotation: turn the robot so tx -> 0 (tag centered) ---
                if (Math.abs(tx) > TX_DEADBAND_DEG) {
                    rotationSpeed = -tx * AIM_GAIN; // negative: turn toward the tag
                    rotationSpeed = clamp(rotationSpeed, -MAX_ROTATION_RAD_PER_SEC, MAX_ROTATION_RAD_PER_SEC);
                }

                // --- Range: drive so ta -> targetArea (same apparent distance as capture time) ---
                double areaError = targetArea - ta; // positive => tag looks smaller now => we're farther => drive forward
                if (Math.abs(areaError) > TA_DEADBAND) {
                    forwardSpeed = areaError * RANGE_GAIN;
                    forwardSpeed = clamp(forwardSpeed, -MAX_FORWARD_MPS, MAX_FORWARD_MPS);
                }
            }
        }
        // If there's no valid target this cycle, forwardSpeed/rotationSpeed simply stay 0
        // (targetArea is left as-is so it re-locks the same distance once the tag reappears).

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(forwardSpeed)
                        .withVelocityY(0.0)
                        .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false; // run until cancelled/interrupted (e.g. bumper released)
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
