package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hoodsubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    // PWMSparkMax uses PWM port number, NOT CAN ID
    // ← change this to your PWM port on the RoboRIO
    private static final int HOOD_PWM_PORT = 0;

    // ── Speed limits ──────────────────────────────────────────────────────────
    // Max power applied to the hood motor (0.0 to 1.0)
    // Lower this if the hood moves too fast and overshoots
    private static final double MAX_POWER = 0.4;

    // ── Simple P gain for position control ───────────────────────────────────
    // Error (in "units") is multiplied by this to get motor output
    // Increase if hood is sluggish, decrease if it oscillates
    private static final double kP = 0.05;

    // ── Tolerance ─────────────────────────────────────────────────────────────
    // How close the hood needs to be to its target before isAtGoal() returns true
    public static final double AT_GOAL_TOLERANCE = 1.0;

    // ── Distance-to-hood-angle interpolation table ────────────────────────────
    // TY → hood target (arbitrary units 0-100, mapped to motor output)
    // Build this from real measurements on the robot.
    //
    //   TY (deg)  |  ~distance  |  target (0-100)
    //   ----------|-------------|----------------
    //    20.0     |   close     |   20
    //    15.0     |   medium    |   40
    //    10.0     |   far       |   65
    //    5.0      |   very far  |   85
    //
    private static final double[][] TY_TO_HOOD = {
        { 20.0, 20.0 },
        { 15.0, 40.0 },
        { 10.0, 65.0 },
        {  5.0, 85.0 },
    };

    // ── Motor ─────────────────────────────────────────────────────────────────
    private final PWMSparkMax hoodMotor;

    // Simulated position — increments/decrements based on motor output each loop
    // This is an estimate since PWMSparkMax has no encoder feedback via WPILib
    private double estimatedPosition = 0.0;
    private double targetPosition    = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public Hoodsubsystem() {
        hoodMotor = new PWMSparkMax(HOOD_PWM_PORT);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        // Simple P loop — drives motor toward target, stops when close enough
        double error = targetPosition - estimatedPosition;

        if (Math.abs(error) < AT_GOAL_TOLERANCE) {
            hoodMotor.set(0);
        } else {
            double output = Math.max(-MAX_POWER, Math.min(MAX_POWER, error * kP));
            hoodMotor.set(output);
            // Update estimated position based on output (tune the 0.5 scale factor)
            estimatedPosition += output * 0.5;
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
        return Math.abs(estimatedPosition - targetPosition) < AT_GOAL_TOLERANCE;
    }

    public double getEstimatedPosition() {
        return estimatedPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    // ── Interpolation ─────────────────────────────────────────────────────────
    private double interpolate(double ty) {
        if (ty >= TY_TO_HOOD[0][0]) return TY_TO_HOOD[0][1];
        if (ty <= TY_TO_HOOD[TY_TO_HOOD.length - 1][0]) return TY_TO_HOOD[TY_TO_HOOD.length - 1][1];

        for (int i = 0; i < TY_TO_HOOD.length - 1; i++) {
            double ty0   = TY_TO_HOOD[i][0],     hood0 = TY_TO_HOOD[i][1];
            double ty1   = TY_TO_HOOD[i + 1][0], hood1 = TY_TO_HOOD[i + 1][1];
            if (ty <= ty0 && ty >= ty1) {
                double t = (ty - ty0) / (ty1 - ty0);
                return hood0 + t * (hood1 - hood0);
            }
        }
        return 0;
    }
}