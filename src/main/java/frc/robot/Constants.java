package frc.robot;

/**
 * All robot-wide numeric constants live here.
 * One place to tune everything -- no more hunting through subsystem files.
 */
public final class Constants {

    /** Swerve drivetrain tuning -- speeds, PID, and current limits */
    public static final class Swerve {
        // Multiplier applied to the max translational speed (1.0 = full speed)
        public static final double kMaxSpeedMultiplier = 1.0;
        // Max rotational speed of the robot in rotations per second
        public static final double kMaxAngularRate = 1.25;
        // Joystick deadband -- inputs below 10% are ignored to prevent drift
        public static final double kDeadbandPercent = 0.1;
        // Speed multiplier when slow-mode is active (left trigger held)
        public static final double kSlowModeMultiplier = 0.2;

        // PathPlanner autonomous path-following PID for translation (X/Y movement)
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;
        // PathPlanner autonomous path-following PID for rotation (heading correction)
        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        // Drive motor closed-loop PID + feedforward (controls wheel speed)
        public static final double kDriveP = 0.2;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.0;   // Static friction feedforward
        public static final double kDriveV = 0.124;  // Velocity feedforward (volts per m/s)

        // Steer motor closed-loop PID + feedforward (controls wheel angle)
        public static final double kSteerP = 100.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.5;
        public static final double kSteerS = 0.1;    // Static friction feedforward
        public static final double kSteerV = 2.66;   // Velocity feedforward
        public static final double kSteerA = 0.0;    // Acceleration feedforward

        // Supply current limits to protect motors and breakers (amps)
        public static final double kDriveCurrentLimit = 80.0;
        public static final double kSteerCurrentLimit = 60.0;
        // Current at which the wheels start to slip -- used for traction control
        public static final double kSlipCurrent = 85.0;
    }

    /** Intake subsystem -- the arm that picks up balls from the ground */
    public static final class Intake {
        // CAN bus IDs for the intake motors
        public static final int kPivotMotorID = 22;   // Motor that raises/lowers the intake arm
        public static final int kRollerMotorID = 31;   // Motor that spins the rollers to grab balls

        // PID gains for the pivot motor position control
        public static final double kPivotP = 4.0;      // Proportional gain
        public static final double kPivotD = 0.1;      // Derivative gain (dampens oscillation)

        // Open-loop speeds for manually raising/lowering the arm
        public static final double kPivotUpSpeed = -0.15;   // Negative = retract up
        public static final double kPivotDownSpeed = 0.15;   // Positive = extend down

        // Roller motor speed (negative = intake direction, full power)
        public static final double kRollerSpeed = -1.0;
    }

    /** Shooter subsystem -- the flywheel that launches balls */
    public static final class Shooter {
        // CAN bus ID for the shooter flywheel motor
        public static final int kShooterMotorID = 24;

        // How many seconds to go from 0 to full output (prevents current spikes)
        public static final double kOpenLoopRampPeriod = 2.0;

        // Velocity closed-loop PID for maintaining consistent flywheel speed
        public static final double kP = 0.6;       // Proportional gain
        public static final double kV = 0.10;      // Velocity feedforward (volts per RPS)

        // Target flywheel velocity in rotations per second
        public static final double kShooterVelocity = 54.0;
    }

    /** Sorter subsystem -- moves balls from intake toward the feeder */
    public static final class Sorter {
        // CAN bus ID for the sorter motor
        public static final int kSorterMotorID = 32;

        // Open-loop duty-cycle speeds (-1 to 1)
        public static final double kForwardSpeed = -0.7;   // Feeds ball toward shooter
        public static final double kReverseSpeed = 0.7;     // Ejects ball backward
    }

    /** Feeder subsystem -- final stage that pushes the ball into the shooter */
    public static final class Feeder {
        // CAN bus ID for the feeder motor
        public static final int kFeederMotorID = 40;

        // Open-loop duty-cycle speeds (-1 to 1)
        public static final double kForwardSpeed = -0.3;   // Feeds ball into shooter wheels
        public static final double kReverseSpeed = 0.8;     // Reverses ball out of feeder
    }

    /** Hood subsystem -- adjusts the launch angle based on distance to target */
    public static final class Hood {
        // RoboRIO PWM port for the hood motor (PWM, not CAN)
        public static final int kHoodPWMPort = 0;

        // Max motor output (0-1) to prevent the hood from moving too fast
        public static final double kMaxPower = 0.4;
        // Proportional gain for the hood position P-loop
        public static final double kP = 0.05;
        // How close (in position units) the hood must be to its target to count as "on target"
        public static final double kAtGoalTolerance = 1.0;
        // Scale factor for estimating hood position from motor output each loop
        public static final double kPositionScaleFactor = 0.5;

        // Lookup table: Limelight TY (vertical angle to target) -> hood position (0-100)
        // Lower TY = farther away = higher hood angle needed
        //   TY (deg)  |  ~distance  |  hood target
        //   20.0      |  close      |  20
        //   15.0      |  medium     |  40
        //   10.0      |  far        |  65
        //    5.0      |  very far   |  85
        public static final double[][] kTYToHood = {
            { 20.0, 20.0 },
            { 15.0, 40.0 },
            { 10.0, 65.0 },
            {  5.0, 85.0 },
        };
    }

    /** Auto-align command -- uses Limelight to rotate the robot toward an AprilTag */
    public static final class AutoAlign {
        // Proportional gain for rotation correction (higher = snappier aim)
        public static final double kRotateKP = 0.04;
        // How many degrees off-center TX can be before we consider alignment "done"
        public static final double kTXTolerance = 3.0;

        // NetworkTables name of the Limelight used for aiming
        public static final String kLimelightName = "limelight";
    }

    /** Timeout values (seconds) used in auto named commands and teleop sequences */
    public static final class Timeouts {
        // --- Autonomous named-command timeouts ---
        // How long the intake arm moves down before stopping
        public static final double kIntakeDownTimeout = 0.2;
        // How long the intake arm moves up before stopping
        public static final double kIntakeUpTimeout = 0.75;
        // Phase 1: run rollers briefly before locking position
        public static final double kRunIntakePhase1Timeout = 0.5;
        // Phase 2: run rollers with arm locked down to finish collecting
        public static final double kRunIntakePhase2Timeout = 5.5;
        // Time allowed for the shooter flywheel to spin up to speed
        public static final double kSpinShooterTimeout = 1.5;
        // Time allowed for the feeder to push the ball into the shooter
        public static final double kFeedBallTimeout = 1.0;
        // Spin-up phase of the full auto shoot sequence
        public static final double kShootSpinUpTimeout = 1.5;
        // Feed phase of the full auto shoot sequence (sorter + feeder + shooter all running)
        public static final double kShootFeedTimeout = 7.0;
        // Max time for the auto-align command during autonomous
        public static final double kAutoAlignTimeout = 2.5;

        // --- Teleop shoot-pipeline timeouts ---
        // Max time for auto-align during the right-bumper shoot pipeline
        public static final double kShootPipelineAlignTimeout = 3.0;
        // Max time for the feed stage after alignment locks on
        public static final double kShootPipelineFeedTimeout = 2.0;
        // Spin-up time for the DPad-left manual shoot sequence
        public static final double kManualShootSpinUpTimeout = 1.5;
    }

    /** Agitate command -- rocks the intake arm back and forth to shake loose jammed balls */
    public static final class Agitate {
        // Seconds between switching the intake arm direction (up <-> down)
        public static final double kSwitchInterval = 0.3;
    }

    /** Operator Interface -- controller USB port assignments */
    public static final class OI {
        // Driver station USB port for the main driver controller
        public static final int kDriverControllerPort = 0;
        // Driver station USB port for the second operator controller
        public static final int kSecondControllerPort = 1;
    }
}
