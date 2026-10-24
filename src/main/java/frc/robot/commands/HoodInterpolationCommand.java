package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/*          
 * Sets the hood angle from the distance to the goal using a straight line between two points:
 *
 *      (startDistM, startHoodDeg)  ----  (endDistM, endHoodDeg)
 *
 * Closer than startDistM -> startHoodDeg. Further than endDistM -> endHoodDeg (clamped, it never
 * extrapolates past your two measured points).
 *
 * The four numbers are published to SmartDashboard under "HoodInterp/..." and re-read every loop,
 * so you can change them live while testing, then hard-code the final values below.
 *
 * NOTE: this requires the Shooter subsystem. Flywheel and hood live in the same subsystem, so
 * scheduling this command will CANCEL ShooterCommand (and its end() stops the flywheel). Use it
 * on its own while you calibrate the hood, then merge the hood line into AutoShooterCommand.
 */

public class HoodInterpolationCommand extends Command {

    // ---------- SET THESE (defaults, editable live from the dashboard) ----------
    private static final double kStartDistM = 1.5;
    private static final double kStartHoodDeg = 8.0;
    private static final double kEndDistM = 4.5;
    private static final double kEndHoodDeg = 28.0;
    // ----------------------------------------------------------------------------

    private final Shooter shooter;
    private final Vision vision;

    public HoodInterpolationCommand(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        SmartDashboard.setDefaultNumber("HoodInterp/StartDistM", kStartDistM);
        SmartDashboard.setDefaultNumber("HoodInterp/StartHoodDeg", kStartHoodDeg);
        SmartDashboard.setDefaultNumber("HoodInterp/EndDistM", kEndDistM);
        SmartDashboard.setDefaultNumber("HoodInterp/EndHoodDeg", kEndHoodDeg);
        addRequirements(shooter);
    }

    /** The one place the distance -> hood mapping lives. Swap this later for the tree map. */
    private double hoodForDistance(double distM) {
        double d0 = SmartDashboard.getNumber("HoodInterp/StartDistM", kStartDistM);
        double h0 = SmartDashboard.getNumber("HoodInterp/StartHoodDeg", kStartHoodDeg);
        double d1 = SmartDashboard.getNumber("HoodInterp/EndDistM", kEndDistM);
        double h1 = SmartDashboard.getNumber("HoodInterp/EndHoodDeg", kEndHoodDeg);

        double span = d1 - d0;
        double t = Math.abs(span) < 1e-6 ? 0.0 : (distM - d0) / span;
        t = MathUtil.clamp(t, 0.0, 1.0);
        return MathUtil.interpolate(h0, h1, t);

        // LATER (tree map): delete everything above and use the table already in Constants:
        // return frc.robot.Constants.ShotTable.hoodDeg.get(distM);
    }

    @Override
    public void execute() {
        double dist = vision.getDistanceToGoal();
        double target = hoodForDistance(dist);
        shooter.setHoodPosition(target); // Shooter clamps to 2.5..38 deg

        SmartDashboard.putNumber("HoodInterp/DistM", dist);
        SmartDashboard.putNumber("HoodInterp/TargetDeg", target);
    }

    @Override
    public void end(boolean interrupted) {
        // Leave the hood where it is; stowing is the job of whoever stops the shooter.
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}