package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * Uses MegaTag2 to get the robot's field position, picks the correct hub
 * based on alliance, and rotates the front of the robot to face that hub.
 */
public class AutoAlignCommand extends Command {

    private static final Translation2d BLUE_HUB =
        new Translation2d(Constants.AutoAlign.kBlueTargetX, Constants.AutoAlign.kBlueTargetY);
    private static final Translation2d RED_HUB =
        new Translation2d(Constants.AutoAlign.kRedTargetX, Constants.AutoAlign.kRedTargetY);

    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem           hood;
    private final double                  maxAngularRate;
    private final boolean                 controlHood;

    private final SwerveRequest.FieldCentric rotateRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private double previousError = 0.0;
    private double lastHeadingError = Double.MAX_VALUE;
    private boolean lastPoseValid = false;

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood) {
        this(drivetrain, hood, true);
    }

    /** @param controlHood false to skip hood adjustment (e.g. pop shot sets hood separately) */
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, boolean controlHood) {
        this.drivetrain     = drivetrain;
        this.hood           = hood;
        this.controlHood    = controlHood;
        this.maxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRate)
                                                .in(RadiansPerSecond);
        addRequirements(drivetrain);
    }

    /**
     * Returns the correct hub target based on alliance color.
     * Defaults to blue if alliance is unknown.
     */
    public static Translation2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB;
        }
        return BLUE_HUB;
    }

    @Override
    public void initialize() {
        previousError = 0.0;
        lastHeadingError = Double.MAX_VALUE;
        lastPoseValid = false;
        if (controlHood) {
            double dist = getDistanceToTarget();
            if (dist > 0) {
                hood.setFromDistance(dist);
            }
        }
    }

    @Override
    public void execute() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );

        lastPoseValid = pose != null && pose.tagCount > 0;
        SmartDashboard.putBoolean("AutoAlign/PoseValid", lastPoseValid);

        if (!lastPoseValid) {
            drivetrain.setControl(brake);
            SmartDashboard.putString("AutoAlign/Status", "No tags — braking");
            lastHeadingError = Double.MAX_VALUE;
            return;
        }

        // Position from vision, but heading from the gyro (much more stable)
        Translation2d robotPos = pose.pose.getTranslation();
        double currentAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        // Pick hub based on alliance
        Translation2d target = getTargetHub();

        // Angle from robot to hub center
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        double error = MathUtil.inputModulus(
            targetAngleDeg - currentAngleDeg + Constants.AutoAlign.kHeadingOffset, -180, 180);
        lastHeadingError = error;

        // Telemetry — check these on SmartDashboard to debug
        SmartDashboard.putNumber("AutoAlign/RobotX",       robotPos.getX());
        SmartDashboard.putNumber("AutoAlign/RobotY",       robotPos.getY());
        SmartDashboard.putNumber("AutoAlign/CurrentAngle",  currentAngleDeg);
        SmartDashboard.putNumber("AutoAlign/TargetX",       target.getX());
        SmartDashboard.putNumber("AutoAlign/TargetY",       target.getY());
        SmartDashboard.putNumber("AutoAlign/TargetAngle",   targetAngleDeg);
        SmartDashboard.putNumber("AutoAlign/HeadingError",  error);
        SmartDashboard.putString("AutoAlign/Status",
            Math.abs(error) < Constants.AutoAlign.kAngleTolerance ? "Aligned!" : "Aligning...");

        // PD control with deadband to prevent jitter
        double rotationOutput;
        if (Math.abs(error) < Constants.AutoAlign.kRotateDeadband) {
            rotationOutput = 0.0;
        } else {
            double derivative = (error - previousError) / 0.02; // 20ms loop
            rotationOutput = (error * Constants.AutoAlign.kRotateKP
                            + derivative * Constants.AutoAlign.kRotateKD)
                            * maxAngularRate;
        }
        previousError = error;

        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationOutput)
        );

        double dist = Math.sqrt(dx * dx + dy * dy);
        SmartDashboard.putNumber("AutoAlign/DistanceMeters", dist);
        if (controlHood) {
            hood.setFromDistance(dist);
        }
    }

    @Override
    public boolean isFinished() {
        // Use cached data from execute() — no redundant Limelight call
        if (!lastPoseValid) return false;
        return Math.abs(lastHeadingError) < Constants.AutoAlign.kAngleTolerance
            && hood.isAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        SmartDashboard.putString("AutoAlign/Status", interrupted ? "Interrupted" : "Done");
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    public static double getDistanceToTarget() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );
        if (pose == null || pose.tagCount == 0) return -1;

        Translation2d robotPos = pose.pose.getTranslation();
        return robotPos.getDistance(getTargetHub());
    }
}
