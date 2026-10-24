package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import java.util.function.Supplier;

/**
 * Hold a button -> the robot turns until the AprilTag sits at the centre of the camera view
 * (tx = 0). The driver keeps full control of translation.
 *
 * - Tag visible:   rotate using tx (camera-relative, so it is not affected by odometry drift).
 * - Tag not seen:  fall back to turning toward the goal using the odometry pose, so the robot
 *                  still points roughly the right way and picks the tag up again.
 *
 * Sign convention: Limelight tx is POSITIVE when the tag is to the RIGHT of the crosshair.
 * WPILib rotation is POSITIVE counter-clockwise. Tag on the right -> robot must turn clockwise
 * (negative), which is what PIDController(measurement = tx, setpoint = 0) gives us.
 */
public class AprilTagCentreCommand extends Command {

    // ---------- TUNE THESE ----------
    private static final double kTxP = 2.0;            // rad/s of turn per rad of tx error
    private static final double kTxD = 0.1;
    private static final double kHeadingP = 3.0;       // odometry fallback, rad/s per rad
    private static final double kMaxOmegaRadPerSec = 3.0;
    private static final double kTxToleranceDeg = 1.0; // "centred" if |tx - offset| < this
    private static final double kTxOffsetDeg = 0.0;    // camera not on shooter axis? put the
                                                       // tx you get when correctly lined up
    // --------------------------------

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Supplier<Translation2d> driverVelocity; // field-relative m/s (x fwd, y left)
    private final int[] allowedTagIds;                    // empty = any tag

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController txPid = new PIDController(kTxP, 0.0, kTxD);
    private final PIDController headingPid = new PIDController(kHeadingP, 0.0, 0.0);

    private boolean centred = false;

    /**
     * @param driverVelocity   supplies the driver's desired field-relative velocity in m/s
     * @param allowedTagIds    only centre on these tag IDs (leave empty to accept any tag)
     */
    public AprilTagCentreCommand(CommandSwerveDrivetrain drivetrain, Vision vision,
                                 Supplier<Translation2d> driverVelocity, int... allowedTagIds) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driverVelocity = driverVelocity;
        this.allowedTagIds = allowedTagIds;
        headingPid.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        txPid.reset();
        headingPid.reset();
        centred = false;
    }

    @Override
    public void execute() {
        double omega;
        boolean usingTag = vision.hasTarget() && tagAllowed(vision.getSeenTagId());

        if (usingTag) {
            double txErrDeg = vision.getTxDeg() - kTxOffsetDeg;
            centred = Math.abs(txErrDeg) < kTxToleranceDeg;
            // measurement = tx error (rad), setpoint = 0  ->  output = -kP * tx
            omega = centred ? 0.0 : txPid.calculate(Math.toRadians(txErrDeg), 0.0);
        } 
        else {
            centred = false;
            txPid.reset();
            double heading = drivetrain.getState().Pose.getRotation().getRadians();
            omega = headingPid.calculate(heading, vision.getAngleToGoal().getRadians());
        }

        omega = MathUtil.clamp(omega, -kMaxOmegaRadPerSec, kMaxOmegaRadPerSec);

        Translation2d v = driverVelocity.get();
        drivetrain.setControl(request
                .withVelocityX(v.getX())
                .withVelocityY(v.getY())
                .withRotationalRate(omega));

        SmartDashboard.putBoolean("TagCentre/UsingTag", usingTag);
        SmartDashboard.putBoolean("TagCentre/Centred", centred);
        SmartDashboard.putNumber("TagCentre/TxDeg", vision.getTxDeg());
        SmartDashboard.putNumber("TagCentre/OmegaCmd", omega);
    }

    /** True while the tag is visible and within tolerance. Use it to gate feeding the shooter. */
    public boolean isCentred() {
        return centred;
    }

    private boolean tagAllowed(int id) {
        if (allowedTagIds.length == 0) return true;
        for (int a : allowedTagIds) if (a == id) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // The default drive command takes over again; nothing to clean up.
    }

    @Override
    public boolean isFinished() {
        return false; // runs while the button is held (use whileTrue)
    }
}