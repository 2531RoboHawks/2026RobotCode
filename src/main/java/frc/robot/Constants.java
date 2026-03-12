package frc.robot;

public final class Constants {
    public static final class Swerve {
        // Speed limits
        public static final double kMaxSpeedMultiplier = 1.0;
        public static final double kMaxAngularRate = 1.25; // rotations per second
        public static final double kDeadbandPercent = 0.1;  // 10%
        public static final double kSlowModeMultiplier = 0.2;

        // PathPlanner PID
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;
        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        // Drive motor PID (Slot 0)
        public static final double kDriveP = 0.2;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.0;
        public static final double kDriveV = 0.124;

        // Steer motor PID (Slot 0)
        public static final double kSteerP = 100.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.5;
        public static final double kSteerS = 0.1;
        public static final double kSteerV = 2.66;
        public static final double kSteerA = 0.0;

        // Current limits
        public static final double kDriveCurrentLimit = 80.0;
        public static final double kSteerCurrentLimit = 60.0;
        public static final double kSlipCurrent = 85.0;
    }
}
