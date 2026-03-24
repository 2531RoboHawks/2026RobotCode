package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hoodsubsystem extends SubsystemBase {

    private final TalonFX            hoodMotor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    private double targetRotations = 0.0;

    public Hoodsubsystem() {
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

        // Press this button on SmartDashboard to snapshot current values to driver station console
        if (SmartDashboard.getBoolean("Hood/Capture", false)) {
            SmartDashboard.putBoolean("Hood/Capture", false);
            String snap = "MotorRotation:" + motorRot + " CurrentRotation:" + current + " Target:" + targetRotations;
            System.out.println(snap);
            SmartDashboard.putString("Hood/Snapshot", snap);
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