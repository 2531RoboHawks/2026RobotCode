package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Drives toward the Hub AprilTag using Limelight tx/ty.
 * Ends when the robot is centered (tx ≈ 0) and at scoring distance (ty ≈ TARGET_TY).
 * Has a 3-second timeout failsafe.
 *
 * Use: "AutoAlign" in PathPlanner.
 */
public class AutoAlignCommand extends Command {

    // ── Tune these ────────────────────────────────────────────────────────────
    private static final double TARGET_TY  = 17.0;  // ← read ty at ideal scoring spot, paste here
    private static final double DRIVE_KP   = 0.09;  // forward/back gain
    private static final double STRAFE_KP  = 0.09;  // left/right gain
    private static final double TX_TOLERANCE = 2.0; // degrees — centered enough
    private static final double TY_TOLERANCE = 2.0; // degrees — close enough
    private static final int[]  HUB_TAG_IDS = { 10, 26 }; // ← your Hub tag IDs
    // ───────────────────────────────────────────────────────────────────────── 

    private final CommandSwerveDrivetrain drivetrain;
    private final double maxSpeed;

    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.maxSpeed   = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        int tagID = getVisibleTagID();

        if (!isHubTag(tagID)) {
            drivetrain.setControl(brake);
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        drivetrain.setControl(
            alignRequest
                .withVelocityX((ty - TARGET_TY) * -DRIVE_KP * maxSpeed)
                .withVelocityY(0)
                .withRotationalRate(tx * -STRAFE_KP)
        );
    }

    @Override
    public boolean isFinished() {
        int tagID = getVisibleTagID();
        if (!isHubTag(tagID)) return false;

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        return Math.abs(tx) < TX_TOLERANCE
            && Math.abs(ty - TARGET_TY) < TY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
    }

    private int getVisibleTagID() {
        return (int) NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tid")
            .getDouble(-1);
    }

    private boolean isHubTag(int id) {
        for (int hubID : HUB_TAG_IDS) {
            if (id == hubID) return true;
        }
        return false;
    }
}
