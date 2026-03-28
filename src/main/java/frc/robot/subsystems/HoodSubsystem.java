package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoAlignCommand;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX            hoodMotor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    private double targetRotations = 0.0;

    public HoodSubsystem() {
        hoodMotor = new TalonFX(Constants.Hood.kHoodMotorID, Constants.Hood.kCANbus);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // PID + feedforward (slot 0)
        cfg.Slot0.kP = Constants.Hood.kP;
        cfg.Slot0.kI = Constants.Hood.kI;
        cfg.Slot0.kD = Constants.Hood.kD;
        cfg.Slot0.kS = Constants.Hood.kS;
        cfg.Slot0.kV = Constants.Hood.kV;
        cfg.Slot0.kG = Constants.Hood.kG;

        // Motion Magic
        cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.kMMCruiseVelocity;
        cfg.MotionMagic.MotionMagicAcceleration   = Constants.Hood.kMMAcceleration;
        cfg.MotionMagic.MotionMagicJerk            = Constants.Hood.kMMJerk;

        // Gear ratio — tells Phoenix 6 to report in mechanism rotations, not motor rotations
        // Two 5:1 reductions = 25:1 total
        cfg.Feedback.SensorToMechanismRatio = Constants.Hood.kGearRatio;

        // Soft limits in mechanism rotations
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Hood.kMaxRotations;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Hood.kMinRotations;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodMotor.getConfigurator().apply(cfg);

        // Zero encoder at startup — robot must start with hood at minimum position
        hoodMotor.setPosition(0.0);

        // Initialize capture button on SmartDashboard
        SmartDashboard.putBoolean("Hood/Capture", false);
        SmartDashboard.putString("Hood/Snapshot", "");
    }

    @Override
    public void periodic() {
        double current = getCurrentRotations();
        double motorRot = current * Constants.Hood.kGearRatio;
        SmartDashboard.putNumber("Hood/TargetRotations",  targetRotations);
        SmartDashboard.putNumber("Hood/CurrentRotations", current);
        SmartDashboard.putBoolean("Hood/AtGoal",          isAtGoal());
        SmartDashboard.putNumber("Hood/MotorRotations",   motorRot);

        // Press this button on SmartDashboard to snapshot auto-align tuning data
        
        if (SmartDashboard.getBoolean("Hood/Capture", false)) {
            SmartDashboard.putBoolean("Hood/Capture", false);
            captureAutoAlignData();
        }
    }

    /**
     * Set hood position from real distance to target in meters.
     * Interpolates the kDistToHood table from Constants.
     */
    public void setFromDistance(double distanceMeters) {
        targetRotations = interpolate(distanceMeters);
        hoodMotor.setControl(motionMagic.withPosition(targetRotations));
    }

    /**
     * Directly command a rotation target (useful for manual tuning).
     */
    public void setPosition(double rotations) {
        targetRotations = Math.max(Constants.Hood.kMinRotations,
                          Math.min(Constants.Hood.kMaxRotations, rotations));
        hoodMotor.setControl(motionMagic.withPosition(targetRotations));
    }

    /**
     * Stow to minimum position (safe for driving).
     */
    public void stow() {
        setPosition(Constants.Hood.kMinRotations);
    }

    public boolean isAtGoal() {
        return Math.abs(getCurrentRotations() - targetRotations) < Constants.Hood.kAtGoalTolerance;
    }

    public double getCurrentRotations() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    // Alias used by RobotContainer for manual dpad adjustment
    public double getTargetPosition() {
        return targetRotations;
    }

    // ── Auto-Align Data Capture ─────────────────────────────────────────────

    /**
     * Snapshots all relevant auto-align tuning data to the console and SmartDashboard.
     * Call from SmartDashboard capture button or operator right trigger.
     */
    public void captureAutoAlignData() {
        double hoodCurrent = getCurrentRotations();
        double hoodMotorRot = hoodCurrent * Constants.Hood.kGearRatio;

        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );

        StringBuilder sb = new StringBuilder();
        sb.append("\n=== AUTO-ALIGN CAPTURE ===\n");

        if (pose == null || pose.tagCount == 0) {
            sb.append("Limelight: NO TAGS DETECTED\n");
        } else {
            Translation2d robotPos = pose.pose.getTranslation();
            double headingDeg = pose.pose.getRotation().getDegrees();

            Translation2d target = AutoAlignCommand.getTargetHub();

            double dx = target.getX() - robotPos.getX();
            double dy = target.getY() - robotPos.getY();
            double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            double error = MathUtil.inputModulus(
                targetAngleDeg - headingDeg + Constants.AutoAlign.kHeadingOffset, -180, 180);
            double distance = Math.sqrt(dx * dx + dy * dy);

            sb.append(String.format("Robot: X=%.3f Y=%.3f Heading=%.1f°\n", robotPos.getX(), robotPos.getY(), headingDeg));
            sb.append(String.format("Target: X=%.3f Y=%.3f\n", target.getX(), target.getY()));
            sb.append(String.format("TargetAngle=%.1f° HeadingError=%.1f°\n", targetAngleDeg, error));
            sb.append(String.format("Distance=%.3fm\n", distance));
            sb.append(String.format("TagCount=%d\n", pose.tagCount));
        }

        sb.append(String.format("Hood: Current=%.4f Target=%.4f Motor=%.2f AtGoal=%b\n",
            hoodCurrent, targetRotations, hoodMotorRot, isAtGoal()));
        sb.append(String.format("PD: kP=%.4f kD=%.4f Offset=%.1f° Deadband=%.1f° Tolerance=%.1f°\n",
            Constants.AutoAlign.kRotateKP, Constants.AutoAlign.kRotateKD,
            Constants.AutoAlign.kHeadingOffset, Constants.AutoAlign.kRotateDeadband,
            Constants.AutoAlign.kAngleTolerance));
        sb.append("===========================");

        String snap = sb.toString();
        System.out.println(snap);
        SmartDashboard.putString("Hood/Snapshot", snap);
    }

    // ── Homing ────────────────────────────────────────────────────────────────

    /**
     * Returns a command that slowly drives the hood down until it stalls,
     * then zeros the encoder. Press once in the pits — do NOT run during a match.
     */
    public Command homeCommand() {
        final int[] stallCount = {0};
        final DutyCycleOut homingControl = new DutyCycleOut(0);

        return runEnd(
            () -> {
                // Drive slowly toward hard stop
                hoodMotor.setControl(homingControl.withOutput(Constants.Hood.kHomingSpeed));

                double current = hoodMotor.getStatorCurrent().getValueAsDouble();
                SmartDashboard.putNumber("Hood/HomingCurrent", current);

                if (current > Constants.Hood.kHomingCurrentThreshold) {
                    stallCount[0]++;
                } else {
                    stallCount[0] = 0;
                }

                if (stallCount[0] >= Constants.Hood.kHomingStallLoops) {
                    // We hit the hard stop — set position so that the offset point becomes 0
                    hoodMotor.setControl(homingControl.withOutput(0));
                    hoodMotor.setPosition(-Constants.Hood.kHomingZeroOffset);
                    targetRotations = 0.0;
                    SmartDashboard.putString("Hood/HomeStatus", "Homed!");
                    System.out.println("Hood homed at offset " + Constants.Hood.kHomingZeroOffset);
                }
            },
            () -> {
                // End: stop motor, reset stall counter
                hoodMotor.setControl(homingControl.withOutput(0));
                stallCount[0] = 0;
            }
        ).until(() -> stallCount[0] >= Constants.Hood.kHomingStallLoops)
         .beforeStarting(() -> {
            // Disable reverse soft limit so we can drive past 0
            SoftwareLimitSwitchConfigs softCfg = new SoftwareLimitSwitchConfigs();
            softCfg.ReverseSoftLimitEnable = false;
            softCfg.ForwardSoftLimitEnable = true;
            softCfg.ForwardSoftLimitThreshold = Constants.Hood.kMaxRotations;
            hoodMotor.getConfigurator().apply(softCfg);
            stallCount[0] = 0;
            SmartDashboard.putString("Hood/HomeStatus", "Homing...");
         })
         .finallyDo((interrupted) -> {
            // Re-enable reverse soft limit
            SoftwareLimitSwitchConfigs softCfg = new SoftwareLimitSwitchConfigs();
            softCfg.ReverseSoftLimitEnable = true;
            softCfg.ReverseSoftLimitThreshold = Constants.Hood.kMinRotations;
            softCfg.ForwardSoftLimitEnable = true;
            softCfg.ForwardSoftLimitThreshold = Constants.Hood.kMaxRotations;
            hoodMotor.getConfigurator().apply(softCfg);
            SmartDashboard.putString("Hood/HomeStatus",
                interrupted ? "Homing interrupted" : "Homed!");
         });
    }

    // ── Interpolation ─────────────────────────────────────────────────────────
    private double interpolate(double distance) {
        double[][] table = Constants.Hood.kDistToHood;

        if (distance <= table[0][0]) return table[0][1];
        if (distance >= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            double d0    = table[i][0],     hood0 = table[i][1];
            double d1    = table[i + 1][0], hood1 = table[i + 1][1];
            if (distance >= d0 && distance <= d1) {
                double t = (distance - d0) / (d1 - d0);
                return hood0 + t * (hood1 - hood0);
            }
        }
        return Constants.Hood.kMinRotations;
    }
}