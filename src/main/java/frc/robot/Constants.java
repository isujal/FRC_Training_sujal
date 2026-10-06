package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        public static final double kFieldLength = 16.54;
        public static final Pose2d kBlueGoal = new Pose2d(4.6, 4.02, new Rotation2d());
    }
}