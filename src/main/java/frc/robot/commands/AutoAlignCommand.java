package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoodsubsystem;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

/**
 * Rotates the bot to face a specific field coordinate target,
 * using MegaTag2 pose estimation to know where the robot is on the field.
 *
 * How it works:
 *   1. Reads robot position from MegaTag2 (AprilTag-based field localization)
 *   2. Determines the correct target (blue or red) based on alliance color
 *   3. Calculates the angle from robot to target using trig
 *   4. Calculates the distance from robot to target
 *   5. Rotates the bot to face the target angle
 *   6. Adjusts hood based on real distance to target (more accurate than TY)
 *
 * Does NOT drive forward/back or strafe — translation is always zero.
 * isFinished() when heading error is within kAngleTolerance AND hood is at goal.
 */
public class AutoAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Hoodsubsystem           hood;
    private final double                  maxAngularRate;

    private final SwerveRequest.FieldCentric rotateRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, Hoodsubsystem hood) {
        this.drivetrain     = drivetrain;
        this.hood           = hood;
        this.maxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRate)
                                                .in(RadiansPerSecond);
        addRequirements(drivetrain, hood);
    }

    @Override
    public void initialize() {
        // Pre-position hood immediately based on current distance
        double dist = getDistanceToTarget();
        if (dist > 0) {
            hood.setFromDistance(dist);
        }
    }

    @Override
    public void execute() {
        // Get robot's current field position from MegaTag2
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );

        // If MegaTag2 doesn't have a valid pose, brake and wait
        if (pose == null || pose.tagCount == 0) {
            drivetrain.setControl(brake);
            return;
        }

        // Get target coordinates for current alliance
        Translation2d target = getTargetCoordinate();

        // Robot's current position
        Translation2d robotPos = pose.pose.getTranslation();

        // Calculate angle from robot to target (radians, then degrees)
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Current robot heading
        double currentAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        // Heading error — how far we need to rotate
        double error = normalizeAngle(targetAngleDeg - currentAngleDeg);

        // Rotate only — no translation
        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(error * Constants.AutoAlign.kRotateKP * maxAngularRate)
        );

        // Update hood based on real distance to target
        double dist = Math.sqrt(dx * dx + dy * dy);
        hood.setFromDistance(dist);
    }

    @Override
    public boolean isFinished() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );
        if (pose == null || pose.tagCount == 0) return false;

        Translation2d target   = getTargetCoordinate();
        Translation2d robotPos = pose.pose.getTranslation();

        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double currentAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        double error           = normalizeAngle(targetAngleDeg - currentAngleDeg);

        return Math.abs(error) < Constants.AutoAlign.kAngleTolerance
            && hood.isAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        if (interrupted) {
            hood.stow();
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Returns the correct target coordinate based on alliance color.
     * Defaults to blue if alliance is not yet detected.
     */
    private Translation2d getTargetCoordinate() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return new Translation2d(Constants.AutoAlign.kRedTargetX, Constants.AutoAlign.kRedTargetY);
        }
        return new Translation2d(Constants.AutoAlign.kBlueTargetX, Constants.AutoAlign.kBlueTargetY);
    }

    /**
     * Returns the straight-line distance from the robot to the target in meters.
     * Returns -1 if pose is not available.
     */
    private double getDistanceToTarget() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );
        if (pose == null || pose.tagCount == 0) return -1;

        Translation2d target   = getTargetCoordinate();
        Translation2d robotPos = pose.pose.getTranslation();
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Normalizes an angle to the range (-180, 180] degrees.
     * Prevents the robot from spinning the long way around.
     */
    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180)  angleDeg -= 360;
        while (angleDeg <= -180) angleDeg += 360;
        return angleDeg;
    }
}