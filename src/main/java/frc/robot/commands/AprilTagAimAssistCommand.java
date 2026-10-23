package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain; // adjust import to match your project's Tuner X generated class

public class AprilTagAimAssistCommand extends Command {
    // ---- Limelight NetworkTable ----
    private static final NetworkTable limelightTable =
            NetworkTableInstance.getDefault().getTable("limelight-centre"); // change if your LL name differs
    public static double mount_angle = 28.0;
    public static double limeheight_inch = 16;
    public static double goalheight = 48;
    public static double target_offset_angle = 0.0;
    public static double angletogoalinrad = 0.0;
    public static double distanceis = 0.0;
    public static NetworkTableEntry Ty = limelightTable.getEntry("ty");

  // NetworkTable limeliNetworkTable ty = limelightTable.getEntry("Ty");

    // ---- Drivetrain ----
    private final CommandSwerveDrivetrain drivetrain;
    // ---- Joystick inputs (supplied live from RobotContainer each cycle) ----
    private final DoubleSupplier forwardSupplier; // joystick -> m/s
    private final DoubleSupplier strafeSupplier;  // joystick -> m/s

    // Robot-centric request: x = forward/back (m/s), y = strafe (m/s), rot = rad/s
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0);

    // ---- Simple proportional gain for aiming (no PIDController object) ----
    private static final double AIM_GAIN = 0.1; // rad/s of rotation per degree of tx error

    // ---- Output cap for rotation ----
    private static final double MAX_ROTATION_RAD_PER_SEC = Math.toRadians(180.0);

    // Minimum target area so we ignore a barely-visible/noisy tag
    private static final double MIN_TARGET_AREA = 0.1;

    // Small deadband so the robot doesn't jitter when already facing the tag
    private static final double TX_DEADBAND_DEG = 1.0;

      /**
     * @param drivetrain      your swerve drivetrain subsystem
     * @param forwardSupplier supplies desired forward/back speed in m/s (already
     *                        scaled from your joystick axis, deadbanded, etc.)
     * @param strafeSupplier  supplies desired strafe speed in m/s (same as above)
     */
    public AprilTagAimAssistCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Make sure Limelight is in AprilTag pipeline / vision mode when this command starts.
        limelightTable.getEntry("camMode").setNumber(0); // 0 = vision processor mode
    }

    @Override
    public void execute() {
        // --- Translation: always comes straight from the driver ---
        double forwardSpeed = forwardSupplier.getAsDouble();
        double strafeSpeed = strafeSupplier.getAsDouble();

        target_offset_angle = Ty.getDouble(0.0);
        angletogoalinrad = (mount_angle + target_offset_angle) * 3.14 / 180;
        distanceis = (goalheight - limeheight_inch) / Math.tan(angletogoalinrad);
        
        // --- Rotation: automatic, driven by the tag ---
        double rotationSpeed = 0.0;

        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0.0) == 1.0;

        if (hasTarget) {
            double tx = limelightTable.getEntry("tx").getDouble(0.0); // horizontal offset, deg
            double ta = limelightTable.getEntry("ta").getDouble(0.0); // target area, 0-100

            if (ta >= MIN_TARGET_AREA && Math.abs(tx) > TX_DEADBAND_DEG) {
                rotationSpeed = -tx * AIM_GAIN; // negative: turn toward the tag
                rotationSpeed = clamp(rotationSpeed, -MAX_ROTATION_RAD_PER_SEC, MAX_ROTATION_RAD_PER_SEC);
            }
        }
        // If no valid target: rotationSpeed stays 0, so the robot simply holds
        // its current heading while the driver keeps driving normally.

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(forwardSpeed)
                        .withVelocityY(strafeSpeed)
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