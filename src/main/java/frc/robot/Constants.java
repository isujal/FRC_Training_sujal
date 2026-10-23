package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;

public final class Constants {
    private Constants() {}

    public static final class DriveConstants {
        private DriveConstants() {}

        /** Top speed in m/s (from Tuner constants, so there is only one source). */
        public static final double kMaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        /** Top turn rate in rad/s (0.75 rotations per second). */
        public static final double kMaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double kStickDeadband = 0.10;   // 10% of stick travel
        public static final double kSlowModeScale = 0.50;   // hold left trigger
        public static final double kTranslationSlewRate = 12.0; // m/s^2, same as Paraducks
    }

    public static final class FieldConstants {
        private FieldConstants() {}

        // TODO: verify both numbers against the game manual / PathPlanner field.
        // Copied from the Paraducks code, which used two slightly different lengths 
        // (16.54 and 16.54175). Keep exactly one value here and use it everywhere.  
        public static final double kFieldLength = 16.54;    // the team should 
        public static final double kFieldWidth = 8.07; // verify
        public static final Pose2d kBlueGoal = new Pose2d(4.6, 4.02, new Rotation2d());
    }


    public static final class VisionConstants {
    public static final String kCameraName = "limelight-centre";
    public static final double kMaxTagDistM = 4.0;     // ignore far tags
    public static final double kMaxOmegaRadPerSec = Math.PI * 4; // ignore fast spins (blur)
}
// Constants.java
public static final class ShotTable {
    public static final InterpolatingDoubleTreeMap hoodDeg = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap rpm = new InterpolatingDoubleTreeMap();
    static {                      // PLACEHOLDERS: replace with your measured values
        hoodDeg.put(1.5, 8.0);  rpm.put(1.5, 2500.0);
        hoodDeg.put(2.5, 15.0); rpm.put(2.5, 3000.0);
        hoodDeg.put(3.5, 22.0); rpm.put(3.5, 3500.0);
        hoodDeg.put(4.5, 28.0); rpm.put(4.5, 4000.0);
    }
}
// also add inside FieldConstants:
// 
}