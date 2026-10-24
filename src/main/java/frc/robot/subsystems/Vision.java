package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String cam = VisionConstants.kCameraName;
    private final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // pick your game's constant
    private LimelightHelpers.PoseEstimate lastEstimate = null;
    private Pose2d pathTarget = null;

    public Vision(CommandSwerveDrivetrain drivetrain) { this.drivetrain = drivetrain; }

    @Override
    public void periodic() {
        var state = drivetrain.getState();

        // 1) Tell the Limelight our heading BEFORE asking for a pose (MegaTag2 needs it)
        LimelightHelpers.SetRobotOrientation(cam, state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // 2) Get the MegaTag2 pose (always blue-origin field coordinates)
        var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        lastEstimate = null;


        if (est != null && est.tagCount > 0
                && est.avgTagDist < VisionConstants.kMaxTagDistM
                && Math.abs(state.Speeds.omegaRadiansPerSecond) < VisionConstants.kMaxOmegaRadPerSec
                && inField(est.pose)) {
            lastEstimate = est;
            // Trust close, multi-tag views more. Heading std is huge: heading comes from the gyro.
            double xy = (0.5 + 0.3 * est.avgTagDist) / Math.max(1, est.tagCount);
            drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds,
                    VecBuilder.fill(xy, xy, 9_999_999));
        }
        publish();
    }

    private boolean inField(Pose2d p) {
        return p.getX() > 0 && p.getX() < FieldConstants.kFieldLength
            && p.getY() > 0 && p.getY() < FieldConstants.kFieldWidth;
    }

    // ----- Goal 1: tag ID + distance, in field space and robot space -----
    public boolean hasTarget()      { return LimelightHelpers.getTV(cam); }
    public int getSeenTagId()       { return (int) LimelightHelpers.getFiducialID(cam); }

    /** Distance (m) from robot center to a tag, measured on the floor plane. */
    public double getDistanceToTag(int id) {
        Optional<Pose3d> tag = layout.getTagPose(id);
        if (tag.isEmpty()) return Double.NaN;
        return drivetrain.getState().Pose.getTranslation()
                .getDistance(tag.get().toPose2d().getTranslation());
    }

    /** Tag expressed in ROBOT space: X = forward, Y = left (WPILib). */
    public Optional<Pose2d> getTagInRobotSpace(int id) {
        return layout.getTagPose(id).map(t -> t.toPose2d().relativeTo(drivetrain.getState().Pose));
    }

    // ----- Goal distance / angle (works even when no tag is visible) -----
    public Translation2d getGoal() {
        Translation2d blue = FieldConstants.kBlueGoal.getTranslation();
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red;
        // Assumes rotational symmetry; verify for your game
        return red ? new Translation2d(FieldConstants.kFieldLength - blue.getX(),
                                       FieldConstants.kFieldWidth - blue.getY()) : blue;
    }
    public double getDistanceToGoal() {
        return drivetrain.getState().Pose.getTranslation().getDistance(getGoal());
    }
    public Rotation2d getAngleToGoal() {   // field-space direction robot->goal
        return getGoal().minus(drivetrain.getState().Pose.getTranslation()).getAngle();
    }

    /** Horizontal offset to the tag in degrees. Positive = tag is to the RIGHT. */
    public double getTxDeg() { return LimelightHelpers.getTX(cam); }

    // ----- Goal 4 helpers -----
    public void setPathTarget(Pose2d p) { pathTarget = p; }
    public Pose2d getVisionPose() { return lastEstimate == null ? null : lastEstimate.pose; }

    private void publish() {
        SmartDashboard.putBoolean("Vision/HasTag", hasTarget());
        SmartDashboard.putNumber("Vision/TagId", getSeenTagId());
        SmartDashboard.putNumber("Vision/DistToGoal", getDistanceToGoal());
        SmartDashboard.putNumber("Vision/TagCount", lastEstimate == null ? 0 : lastEstimate.tagCount);
        if (lastEstimate != null && pathTarget != null) {
            SmartDashboard.putNumber("Vision/PathErrorM",
                lastEstimate.pose.getTranslation().getDistance(pathTarget.getTranslation()));
        }
    }
}