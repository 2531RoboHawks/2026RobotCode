package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoodsubsystem;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Rotates the bot until the Limelight crosshair is centered on the tag (TX = 0).
 * Does NOT drive forward/back or strafe — translation is zero.
 * Hood adjusts based on TY (distance from tag) in parallel.
 *
 * isFinished() when TX is within ±3° AND hood is at goal.
 */
public class AutoAlignCommand extends Command {

    // ── Rotation gain — tune this if rotation is too fast/slow/oscillates ────
    private static final double ROTATE_KP    = 0.04; // degrees TX → fraction of max angular rate
    private static final double TX_TOLERANCE = 3.0;  // degrees — ±3° is "close enough"

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
        this.maxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
        addRequirements(drivetrain, hood);
    }

    @Override
    public void initialize() {
        // Start moving hood immediately based on current TY
        if (hasTarget()) {
            hood.setFromTY(LimelightHelpers.getTY("limelight"));
        }
    }

    @Override
    public void execute() {
        if (!hasTarget()) {
            drivetrain.setControl(brake);
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        // Only rotate — zero velocity X and Y
        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-tx * ROTATE_KP * maxAngularRate)
        );

        // Continuously update hood as TY changes while rotating
        hood.setFromTY(ty);
    }

    @Override
    public boolean isFinished() {
        if (!hasTarget()) return false;
        return Math.abs(LimelightHelpers.getTX("limelight")) < TX_TOLERANCE
            && hood.isAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
        if (interrupted) {
            hood.stow();
        }
    }

    private boolean hasTarget() {
        return NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tv")
            .getDouble(0) == 1.0;
    }
}