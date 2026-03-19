package frc.robot;

/**
 * All robot-wide numeric constants live here.
 * One place to tune everything -- no more hunting through subsystem files.
 */
public final class Constants {

    /** Swerve drivetrain tuning -- speeds, PID, and current limits */
    public static final class Swerve {
        public static final double kMaxSpeedMultiplier = 1.0;
        public static final double kMaxAngularRate     = 1.25;
        public static final double kDeadbandPercent    = 0.1;
        public static final double kSlowModeMultiplier = 0.2;

        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;
        public static final double kRotationP    = 5.0;
        public static final double kRotationI    = 0.0;
        public static final double kRotationD    = 0.0;

        public static final double kDriveP = 0.2;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.0;
        public static final double kDriveV = 0.124;

        public static final double kSteerP = 100.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.5;
        public static final double kSteerS = 0.1;
        public static final double kSteerV = 2.66;
        public static final double kSteerA = 0.0;

        public static final double kDriveCurrentLimit = 80.0;
        public static final double kSteerCurrentLimit = 60.0;
        public static final double kSlipCurrent       = 85.0;
    }

    /** Intake subsystem */
    public static final class Intake {
        public static final int    kPivotMotorID   = 22;
        public static final int    kRollerMotorID  = 31;
        public static final double kPivotP         = 40.0;
        public static final double kPivotD         = 0.8;
        public static final double kPivotUpSpeed   = -0.15;
        public static final double kPivotDownSpeed = 0.15;
        public static final double kRollerSpeed    = -0.8;

        public static final double kPivotHoldCurrentLimit = 20.0;

    }

    /** Shooter subsystem */
    public static final class Shooter {
        public static final int    kShooterMotorID     = 24;
        public static final double kOpenLoopRampPeriod = 2.0;
        public static final double kP                  = 0.6;
        public static final double kV                  = 0.10;
        public static final double kShooterVelocity    = 54.0;
    }

    /** Sorter subsystem */
    public static final class Sorter {
        public static final int    kSorterMotorID = 32;
        public static final double kForwardSpeed  = -0.7;
        public static final double kReverseSpeed  = 0.7;
    }

    /** Feeder subsystem */
    public static final class Feeder {
        // Amps — tune this up/down based on real current readings with and without a ball
    public static final double kBallDetectCurrentThreshold = 2.0;      
        public static final int    kFeederMotorID = 40;
        public static final double kForwardSpeed  = -0.3;
        public static final double kReverseSpeed  = 0.8;
    }

    /** Hood subsystem -- Falcon 500 with two 5:1 reductions (25:1 total) */
    public static final class Hood {

        // ← Change this to your hood motor's CAN ID
        public static final int    kHoodMotorID  = 33;
        // ← Change to "rio" if not on a CANivore
        public static final String kCANbus        = "rio";

        // Two 5:1 reductions = 25:1 total gear ratio
        // Phoenix 6 uses this to report position in mechanism rotations (not motor rotations)
        public static final double kGearRatio = 25.0;

        // Soft limits in mechanism rotations (output shaft, after gear reduction)
        // kMinRotations should always be 0 (starting position)
        // kMaxRotations: drive hood to physical max, read Hood/CurrentRotations, set that here
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 2.0; // ← tune this

        // How close (mechanism rotations) the hood must be to count as at goal
        public static final double kAtGoalTolerance = 0.05;

        // ── Motion Magic ──────────────────────────────────────────────────────
        // CruiseVelocity: max speed in mechanism rot/s — increase for faster movement
        // Acceleration:   how quickly it reaches cruise — increase for snappier start
        // Jerk:           how quickly acceleration ramps — leave high to avoid jerk limits
        public static final double kMMCruiseVelocity = 2.0;
        public static final double kMMAcceleration   = 2.0;
        public static final double kMMJerk           = 20.0;

        // ── PID + Feedforward ─────────────────────────────────────────────────
        // kP: proportional gain — increase if hood is slow to reach target
        // kD: derivative gain  — increase if hood oscillates
        // kS: static friction  — minimum voltage to overcome friction
        // kV: velocity feedforward — volts per rot/s
        // kG: gravity feedforward — increase if hood sags under gravity, 0 if horizontal
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.1;
        public static final double kV = 0.08;
        public static final double kG = 0.0;

        // ── Distance to hood position table ───────────────────────────────────
        // Each row: { distance_meters, hood_mechanism_rotations }
        // Distance is the straight-line distance from robot to target coordinate.
        //
        // How to tune:
        //   1. Drive to a known distance from the target
        //   2. Hold right bumper — bot aligns, check Hood/CurrentRotations
        //   3. Manually adjust hood until shot lands on target
        //   4. Read Hood/CurrentRotations from SmartDashboard
        //   5. Update that row: { distance, that rotation value }
        //
        //   Distance (m) | hood rotations
        //   -------------|---------------
        //   2.0          | 1.0   (close)
        //   3.0          | 2.5   (medium)
        //   4.0          | 4.5   (far)
        //   5.0          | 7.0   (very far)
        public static final double[][] kDistToHood = {
            { 2.0, 0.5 },
            { 3.0, 1.0 },
            { 4.0, 1.5 },
            { 5.0, 2.0 },
        };
    }

    /** Auto-align and field targeting */
    public static final class AutoAlign {
        // NetworkTables name of the Limelight
        public static final String kLimelightName = "limelight";

        // Proportional gain for rotation — increase if sluggish, decrease if oscillates
        public static final double kRotateKP = 0.01;

        // Degrees of heading error that counts as "aligned"
        // Looser = faster. Tighter = more accurate.
        public static final double kAngleTolerance = 2.5;

        // Blue alliance shooting target (meters, WPILib field coordinates)
        public static final double kBlueTargetX = 4.621;
        public static final double kBlueTargetY = 4.025;

        // Red alliance shooting target (meters, WPILib field coordinates)
        public static final double kRedTargetX = 11.919;
        public static final double kRedTargetY = 4.025;
    }

    /** Timeout values (seconds) */
    public static final class Timeouts {
        public static final double kIntakeDownTimeout         = 0.2;
        public static final double kIntakeUpTimeout           = 0.75;
        public static final double kRunIntakePhase1Timeout    = 0.5;
        public static final double kRunIntakePhase2Timeout    = 5.5;
        public static final double kSpinShooterTimeout        = 1.5;
        public static final double kFeedBallTimeout           = 1.0;
        public static final double kShootSpinUpTimeout        = 1.5;
        public static final double kShootFeedTimeout          = 7.0;
        public static final double kAutoAlignTimeout          = 2.5;
        public static final double kShootPipelineAlignTimeout = 2.0;
        public static final double kShootPipelineFeedTimeout  = 4.0;
        public static final double kManualShootSpinUpTimeout  = 1.5;
    }

    /** Agitate command */
    public static final class Agitate {
        public static final double kSwitchInterval = 0.3;
    }

    /** CANdle LED subsystem */
    public static final class CANdle {
        public static final int CAN_ID = 41;
        public static final String CAN_BUS = "";
        public static final int LED_START = 0;
        public static final int LED_ONBOARD_COUNT = 8;
        public static final int LED_EXTERNAL_COUNT = 60;
        public static final int LED_END = LED_START + LED_ONBOARD_COUNT + LED_EXTERNAL_COUNT - 1;

        public static final String STRIP_TYPE = "GRB";

        public static final String KEY_R = "CANdle/R";
        public static final String KEY_G = "CANdle/G";
        public static final String KEY_B = "CANdle/B";
        public static final String KEY_OFF = "CANdle/Off";
        public static final String KEY_AUTO_MODE = "CANdle/AutoMode";
        public static final String KEY_PURPLE_GOLD = "CANdle/PurpleGold";
        public static final String KEY_STATUS = "CANdle/Status";
        public static final String KEY_CONFIG_STATUS = "CANdle/ConfigStatus";
        public static final String KEY_APPLIED_R = "CANdle/AppliedR";
        public static final String KEY_APPLIED_G = "CANdle/AppliedG";
        public static final String KEY_APPLIED_B = "CANdle/AppliedB";

        public static final int PURPLE_R = 128;
        public static final int PURPLE_G = 0;
        public static final int PURPLE_B = 255;
        public static final int GOLD_R = 255;
        public static final int GOLD_G = 191;
        public static final int GOLD_B = 0;
        public static final double AUTO_CYCLE_SECONDS = 1.5;
    }

    /** Operator Interface */
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kSecondControllerPort = 1;
    }
}