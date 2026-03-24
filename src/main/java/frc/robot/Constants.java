package frc.robot;

/**
 * All robot-wide numeric constants live here.
 * One place to tune everything -- no more hunting through subsystem files.
 */
public final class Constants {

    /** Swerve drivetrain tuning -- speeds, PID, and current limits */
    public static final class Swerve {
        public static final double kMaxSpeedMultiplier = 1.0; // full-speed scale (0–1)
        public static final double kMaxAngularRate     = 1.25; // max turn speed in rot/s (0.75–2.0)
        public static final double kDeadbandPercent    = 0.1; // joystick deadzone % (0.05–0.15)
        public static final double kSlowModeMultiplier = 0.14; // slow mode speed scale (0.15–0.35)

        public static final double kTranslationP = 5.0; // auto path position correction (1–10)
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;
        public static final double kRotationP    = 5.0; // auto path heading correction (1–10)
        public static final double kRotationI    = 0.0;
        public static final double kRotationD    = 0.0;

        public static final double kDriveP = 0.2; // wheel speed correction (0.05–0.5)
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.0; // static friction volts (0–0.25)
        public static final double kDriveV = 0.124; // velocity FF, volts per speed unit (0.1–0.15)

        public static final double kSteerP = 100.0; // wheel angle snap strength (50–150)
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.5; // dampens angle oscillation (0–2)
        public static final double kSteerS = 0.1; // static friction volts (0–0.25)
        public static final double kSteerV = 2.66; // velocity FF (1.5–3.5)
        public static final double kSteerA = 0.0; // accel FF (usually 0)

        public static final double kDriveCurrentLimit = 60.0; // drive motor amps cap (40–80)
        public static final double kSteerCurrentLimit = 60.0; // steer motor amps cap (20–60)
        public static final double kSlipCurrent       = 70.0; // traction control limit amps (60–120)
    }

    /** Intake subsystem */
    public static final class Intake {
        public static final int    kPivotMotorID   = 22; // CAN ID of pivot motor
        public static final int    kRollerMotorID  = 31; // CAN ID of roller motor
        public static final double kPivotP         = 80.0; // position hold strength (20–200)
        public static final double kPivotD         = 5.0; // dampens pivot oscillation (0.5–10)
        public static final double kPivotUpSpeed   = -0.3; // open-loop up power, negative = up (-0.1 to -0.5)
        public static final double kPivotDownSpeed = 0.15; // open-loop down power (0.1–0.3)
        public static final double kRollerSpeed    = -0.8; // roller intake speed (-0.5 to -1.0)

        public static final double kPivotHoldCurrentLimit = 40.0; // max amps for pivot motor (20–60)

    }

    /** Shooter subsystem */
    public static final class Shooter {
        public static final int    kShooterMotorID     = 24; // CAN ID of shooter motor
        public static final double kOpenLoopRampPeriod = 2.0; // seconds to ramp to full speed (0.5–3.0)
        public static final double kP                  = 0.6; // velocity correction strength (0.1–1.0)
        public static final double kV                  = 0.10; // velocity FF, volts per rot/s (0.05–0.15)
        public static final double kShooterVelocity    = 43.0; // target speed in rot/s (30–60)
        public static final double kPopShotVelocity   = 29.0; // close-range pop shot speed
    }

    /** Sorter subsystem */
    public static final class Sorter {
        public static final int    kSorterMotorID = 32; // CAN ID of sorter motor
        public static final double kForwardSpeed  = -0.5; // forward sort power (-0.3 to -0.8)
        public static final double kReverseSpeed  = 0.7; // reverse sort power (0.3–1.0)
    }

    /** Feeder subsystem */
    public static final class Feeder {
        // Amps — tune this up/down based on real current readings with and without a ball
    public static final double kBallDetectCurrentThreshold = 2.0; // current spike = ball detected (1–5 amps)
        public static final int    kFeederMotorID = 40; // CAN ID of feeder motor
        public static final double kForwardSpeed  = -0.5; // feed forward power (-0.3 to -0.8)
        public static final double kReverseSpeed  = 0.8; // feed reverse power (0.5–1.0)
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

        // Hood position for pop shots (mechanism rotations) — slight angle for close range
        public static final double kPopShotHood = 0.12;

        // How close (mechanism rotations) the hood must be to count as at goal
        public static final double kAtGoalTolerance = 0.05;

        // ── Motion Magic ──────────────────────────────────────────────────────
        // CruiseVelocity: max speed in mechanism rot/s — increase for faster movement
        // Acceleration:   how quickly it reaches cruise — increase for snappier start
        // Jerk:           how quickly acceleration ramps — leave high to avoid jerk limits
        public static final double kMMCruiseVelocity = 15.0;
        public static final double kMMAcceleration   = 25.0;
        public static final double kMMJerk           = 100.0;

        // ── PID + Feedforward ─────────────────────────────────────────────────
        // kP: proportional gain — increase if hood is slow to reach target
        // kD: derivative gain  — increase if hood oscillates
        // kS: static friction  — minimum voltage to overcome friction
        // kV: velocity feedforward — volts per rot/s
        // kG: gravity feedforward — increase if hood sags under gravity, 0 if horizontal
        public static final double kP = 3.0;
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
        // TODO: need to fix limelight position
        // Known good: ~3.45m → 0.04 rotations
        public static final double[][] kDistToHood = {
            { 1.0, 0.0 },    // very close — hood flat
            { 2.0, 0.0 },    // close — hood flat
            { 3.0, 0.03 },   // medium
            { 3.5, 0.04 },   // tested value
            { 4.0, 0.055 },  // far — tune on robot
            { 5.0, 0.08 },   // very far — tune on robot
            { 6.0, 0.10 },   // max range — tune on robot
        };
    }

    /** Auto-align and field targeting */
    public static final class AutoAlign {
        // NetworkTables name of the Limelight
        public static final String kLimelightName = "limelight-temp";

        // Proportional gain for rotation — increase if sluggish, decrease if oscillates
        public static final double kRotateKP = 0.015; // rotation correction strength (0.005–0.05)

        // Derivative gain for rotation — dampens oscillation / jitter
        public static final double kRotateKD = 0.002;

        // Degrees of heading error below which rotation output is zeroed (stops jitter)
        public static final double kRotateDeadband = 1.0;

        // TODO: need to fix limelight position
        // Heading offset in degrees — positive = aim further left, negative = aim further right
        // Compensates for limelight being physically misaligned from robot center
        public static final double kHeadingOffset = 9.0; // ← tune on robot (limelight was moved, shooting right — positive shifts aim left)

        // Degrees of heading error that counts as "aligned"
        // Looser = faster. Tighter = more accurate.
        public static final double kAngleTolerance = 2.0; // degrees of allowed error (1–5)

        // Distance (meters) below which a pop shot is used instead of full alignment
        public static final double kPopShotDistance = 2.5;

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
        public static final double kManualShootSpinUpTimeout  = 1.0;
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