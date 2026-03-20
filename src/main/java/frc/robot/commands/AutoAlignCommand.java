package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        addRequirements(drivetrain); // hood not required — avoids default command conflict
    }

    @Override
    public void initialize() {
        double dist = getDistanceToTarget();
        if (dist > 0) {
            hood.setFromDistance(dist);
        }
    }

    @Override
    public void execute() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );

        // ── Debug output — check these on SmartDashboard ──────────────────────
        boolean poseValid = pose != null && pose.tagCount > 0;
        SmartDashboard.putBoolean("AutoAlign/PoseValid",     poseValid);
        SmartDashboard.putNumber("AutoAlign/TagCount",       pose != null ? pose.tagCount : -1);
        SmartDashboard.putNumber("AutoAlign/RobotX",         pose != null ? pose.pose.getX() : -1);
        SmartDashboard.putNumber("AutoAlign/RobotY",         pose != null ? pose.pose.getY() : -1);
        SmartDashboard.putNumber("AutoAlign/CurrentHeading", drivetrain.getState().Pose.getRotation().getDegrees());

        Translation2d target = getTargetCoordinate();
        SmartDashboard.putNumber("AutoAlign/TargetX", target.getX());
        SmartDashboard.putNumber("AutoAlign/TargetY", target.getY());
        SmartDashboard.putString("AutoAlign/Alliance",
            DriverStation.getAlliance().map(a -> a.toString()).orElse("Unknown"));
        // ─────────────────────────────────────────────────────────────────────

        if (!poseValid) {
            // No valid MegaTag2 pose — brake and wait
            drivetrain.setControl(brake);
            SmartDashboard.putNumber("AutoAlign/HeadingError", 0);
            SmartDashboard.putString("AutoAlign/Status", "No pose — braking");
            return;
        }

        Translation2d robotPos = pose.pose.getTranslation();
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double currentAngleDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        double error           = normalizeAngle(targetAngleDeg - currentAngleDeg);

        SmartDashboard.putNumber("AutoAlign/TargetAngle",  targetAngleDeg);
        SmartDashboard.putNumber("AutoAlign/HeadingError", error);
        SmartDashboard.putString("AutoAlign/Status",
            Math.abs(error) < Constants.AutoAlign.kAngleTolerance ? "Aligned!" : "Aligning...");

        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(error * Constants.AutoAlign.kRotateKP * maxAngularRate)
        );

        double dist = Math.sqrt(dx * dx + dy * dy);
        SmartDashboard.putNumber("AutoAlign/DistanceMeters", dist);
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
        SmartDashboard.putString("AutoAlign/Status", interrupted ? "Interrupted" : "Done");
        // hood intentionally NOT stowed here — let the shoot pipeline finish feeding first
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private Translation2d getTargetCoordinate() {
        Translation2d blue = new Translation2d(Constants.AutoAlign.kBlueTargetX, Constants.AutoAlign.kBlueTargetY);
        Translation2d red  = new Translation2d(Constants.AutoAlign.kRedTargetX, Constants.AutoAlign.kRedTargetY);

        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        double distBlue = robotPos.getDistance(blue);
        double distRed  = robotPos.getDistance(red);

        return distRed < distBlue ? red : blue;
    }

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

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180)  angleDeg -= 360;
        while (angleDeg <= -180) angleDeg += 360;
        return angleDeg;
    }
}