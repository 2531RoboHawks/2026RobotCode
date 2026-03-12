package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hoodsubsystem extends SubsystemBase {

    // ── Motor ─────────────────────────────────────────────────────────────────
    private final PWMSparkMax hoodMotor;

    // Simulated position — increments/decrements based on motor output each loop
    // This is an estimate since PWMSparkMax has no encoder feedback via WPILib
    private double estimatedPosition = 0.0;
    private double targetPosition    = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Hoodsubsystem() {
        hoodMotor = new PWMSparkMax(Constants.Hood.kHoodPWMPort);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        // Simple P loop — drives motor toward target, stops when close enough
        double error = targetPosition - estimatedPosition;

        if (Math.abs(error) < Constants.Hood.kAtGoalTolerance) {
            hoodMotor.set(0);
        } else {
            double output = Math.max(-Constants.Hood.kMaxPower,
                Math.min(Constants.Hood.kMaxPower, error * Constants.Hood.kP));
            hoodMotor.set(output);
            estimatedPosition += output * Constants.Hood.kPositionScaleFactor;
        }

        SmartDashboard.putNumber("Hood/TargetPosition",    targetPosition);
        SmartDashboard.putNumber("Hood/EstimatedPosition", estimatedPosition);
        SmartDashboard.putBoolean("Hood/AtGoal",           isAtGoal());
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Set hood position from a Limelight TY reading.
     * Interpolates the TY_TO_HOOD table.
     */
    public void setFromTY(double ty) {
        targetPosition = interpolate(ty);
    }

    /**
     * Directly set a target position (0-100 scale).
     */
    public void setPosition(double position) {
        targetPosition = Math.max(0, Math.min(100, position));
    }

    /**
     * Stow to the minimum position.
     */
    public void stow() {
        setPosition(0);
    }

    public boolean isAtGoal() {
        return Math.abs(estimatedPosition - targetPosition) < Constants.Hood.kAtGoalTolerance;
    }

    public double getEstimatedPosition() {
        return estimatedPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    // ── Interpolation ─────────────────────────────────────────────────────────
    private double interpolate(double ty) {
        double[][] table = Constants.Hood.kTYToHood;
        if (ty >= table[0][0]) return table[0][1];
        if (ty <= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            double ty0   = table[i][0],     hood0 = table[i][1];
            double ty1   = table[i + 1][0], hood1 = table[i + 1][1];
            if (ty <= ty0 && ty >= ty1) {
                double t = (ty - ty0) / (ty1 - ty0);
                return hood0 + t * (hood1 - hood0);
            }
        }
        return 0;
    }
}
