package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoodsubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * Uses MegaTag1 to get the robot's field position, picks whichever hub
 * is closer, and rotates the front of the robot to face that hub's center.
 */
public class AutoAlignCommand extends Command {

    private static final Translation2d BLUE_HUB =
        new Translation2d(Constants.AutoAlign.kBlueTargetX, Constants.AutoAlign.kBlueTargetY);
    private static final Translation2d RED_HUB =
        new Translation2d(Constants.AutoAlign.kRedTargetX, Constants.AutoAlign.kRedTargetY);

    private final CommandSwerveDrivetrain drivetrain;
    private final Hoodsubsystem           hood;
    private final double                  maxAngularRate;
    private final boolean                 controlHood;

    private final SwerveRequest.FieldCentric rotateRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private double previousError = 0.0;

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, Hoodsubsystem hood) {
        this(drivetrain, hood, true);
    }

    /** @param controlHood false to skip hood adjustment (e.g. pop shot sets hood separately) */
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, Hoodsubsystem hood, boolean controlHood) {
        this.drivetrain     = drivetrain;
        this.hood           = hood;
        this.controlHood    = controlHood;
        this.maxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRate)
                                                .in(RadiansPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        previousError = 0.0;
        if (controlHood) {
            double dist = getDistanceToTarget();
            if (dist > 0) {
                hood.setFromDistance(dist);
            }
        }
    }

    @Override
    public void execute() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(
            Constants.AutoAlign.kLimelightName
        );

        boolean poseValid = pose != null && pose.tagCount > 0;
        SmartDashboard.putBoolean("AutoAlign/PoseValid", poseValid);

        if (!poseValid) {
            drivetrain.setControl(brake);
            SmartDashboard.putString("AutoAlign/Status", "No tags — braking");
            return;
        }

        // Robot position AND heading both from MegaTag1 (same coordinate frame)
        Translation2d robotPos = pose.pose.getTranslation();
        double currentAngleDeg = pose.pose.getRotation().getDegrees();

        // Pick whichever hub is closer
        Translation2d target = robotPos.getDistance(RED_HUB) < robotPos.getDistance(BLUE_HUB)
            ? RED_HUB : BLUE_HUB;

        // Angle from robot to hub center
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        double error = normalizeAngle(targetAngleDeg - currentAngleDeg + Constants.AutoAlign.kHeadingOffset);

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
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(
            Constants.AutoAlign.kLimelightName
        );
        if (pose == null || pose.tagCount == 0) return false;

        Translation2d robotPos = pose.pose.getTranslation();
        Translation2d target = robotPos.getDistance(RED_HUB) < robotPos.getDistance(BLUE_HUB)
            ? RED_HUB : BLUE_HUB;

        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        double targetAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double currentAngleDeg = pose.pose.getRotation().getDegrees();
        double error           = normalizeAngle(targetAngleDeg - currentAngleDeg + Constants.AutoAlign.kHeadingOffset);

        return Math.abs(error) < Constants.AutoAlign.kAngleTolerance
            && hood.isAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        SmartDashboard.putString("AutoAlign/Status", interrupted ? "Interrupted" : "Done");
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    public static double getDistanceToTarget() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(
            Constants.AutoAlign.kLimelightName
        );
        if (pose == null || pose.tagCount == 0) return -1;

        Translation2d robotPos = pose.pose.getTranslation();
        Translation2d target = robotPos.getDistance(RED_HUB) < robotPos.getDistance(BLUE_HUB)
            ? RED_HUB : BLUE_HUB;
        return robotPos.getDistance(target);
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180)  angleDeg -= 360;
        while (angleDeg <= -180) angleDeg += 360;
        return angleDeg;
    }
}
