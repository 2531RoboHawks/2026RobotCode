package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Drives toward an AprilTag using Limelight tx/ty.
 * Each tag has its own TX and TY target.
 * To add more tags, just add entries to TAG_TARGETS below.
 *
 * Use: "AutoAlignSai" in PathPlanner.
 */
public class AutoAlignCommand extends Command {

    // ── Gains ─────────────────────────────────────────────────────────────────
    private static final double DRIVE_KP     = 0.09;
    private static final double STRAFE_KP    = 0.09;
    private static final double TX_TOLERANCE = 2.0;
    private static final double TY_TOLERANCE = 2.0;

    // Default targets used if a tag isn't listed below
    private static final double DEFAULT_TY = 17.0;
    private static final double DEFAULT_TX = 0.0;
    // ─────────────────────────────────────────────────────────────────────────

    // ── ADD/EDIT TAGS HERE ────────────────────────────────────────────────────
    // Format: TAG_TARGETS.put(tagID, new double[]{TX, TY});
    // TX = left/right offset from tag center (0 = dead center, + = right, - = left)
    // TY = how far forward to drive (higher = closer to tag)
    public static final Map<Integer, double[]> TAG_TARGETS = new HashMap<>();
    static {
        TAG_TARGETS.put(10, new double[]{ 0.0, 17.0 }); // { TX, TY }
        TAG_TARGETS.put(26, new double[]{ 0.0, 17.0 });
        TAG_TARGETS.put(13, new double[]{ -5.0, 8.0 });
        TAG_TARGETS.put(29, new double[]{ -5.0, 8.0 });

        // Add more tags here — just uncomment and fill in values:
        // TAG_TARGETS.put(7,  new double[]{ 0.0, 15.0 });
        // TAG_TARGETS.put(8,  new double[]{ 0.0, 15.0 });
        // TAG_TARGETS.put(11, new double[]{ 1.5, 16.0 });
        // TAG_TARGETS.put(12, new double[]{ 0.0, 17.0 });
    }
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

    public static double getTargetTYStatic(int tagID) {
    return TAG_TARGETS.containsKey(tagID) ? TAG_TARGETS.get(tagID)[1] : DEFAULT_TY;
}

public static double getTargetTXStatic(int tagID) {
    return TAG_TARGETS.containsKey(tagID) ? TAG_TARGETS.get(tagID)[0] : DEFAULT_TX;
}   

    @Override
    public void execute() {
        int tagID = getVisibleTagID();

        if (!isValidTag(tagID)) {
            drivetrain.setControl(brake);
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        drivetrain.setControl(
            alignRequest
                .withVelocityX((ty - getTargetTY(tagID)) * -DRIVE_KP * maxSpeed)
                .withVelocityY((tx - getTargetTX(tagID)) * -STRAFE_KP * maxSpeed)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        int tagID = getVisibleTagID();
        if (!isValidTag(tagID)) return false;

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        return Math.abs(tx - getTargetTX(tagID)) < TX_TOLERANCE
            && Math.abs(ty - getTargetTY(tagID)) < TY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private double getTargetTX(int tagID) {
        return TAG_TARGETS.containsKey(tagID) ? TAG_TARGETS.get(tagID)[0] : DEFAULT_TX;
    }

    private double getTargetTY(int tagID) {
        return TAG_TARGETS.containsKey(tagID) ? TAG_TARGETS.get(tagID)[1] : DEFAULT_TY;
    }

    private boolean isValidTag(int tagID) {
        return TAG_TARGETS.containsKey(tagID);
    }

    private int getVisibleTagID() {
        return (int) NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tid")
            .getDouble(-1);
    }
}