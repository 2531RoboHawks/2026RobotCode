package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.kShooterMotorID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private static final double AT_SPEED_TOLERANCE = 2.0; // RPS

    private double targetVelocity = 0.0;

    private final GenericEntry velocityEntry;
    private final GenericEntry atSpeedEntry;

    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Shooter.kOpenLoopRampPeriod;

        // Velocity PID — tune kP if wheel bogs down under load
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = Constants.Shooter.kP;
        slot0.kV = Constants.Shooter.kV;

        config.CurrentLimits.StatorCurrentLimit       = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        shooterMotor.getConfigurator().apply(config);

        ShuffleboardTab tab = Shuffleboard.getTab("Important");
        velocityEntry = tab.add("Shooter Velocity (RPS)", 0).getEntry();
        atSpeedEntry  = tab.add("Shooter At Speed", false).getEntry();
    }

    @Override
    public void periodic() {
        double velocity = getVelocity();
        velocityEntry.setDouble(velocity);
        atSpeedEntry.setBoolean(isAtSpeed());
    }

    public double getVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return targetVelocity > 0 && getVelocity() >= (targetVelocity - AT_SPEED_TOLERANCE);
    }

    // Returns a command that runs the shooter while active and stops it when ended
    public Command runShooter() {
        return runEnd(
            () -> runShooterMotor(),
            () -> stopShooter()
        );
    }

    // Spins the shooter wheel using closed-loop velocity control to launch game pieces
    public void runShooterMotor() {
        targetVelocity = Constants.Shooter.kShooterVelocity;
        shooterMotor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    // Pop shot velocity for close-range shots
    public void runPopShotMotor() {
        targetVelocity = Constants.Shooter.kPopShotVelocity;
        shooterMotor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    // Long-range velocity for shots >4m
    public void runLongRangeMotor() {
        targetVelocity = Constants.Shooter.kLongRangeVelocity;
        shooterMotor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    // Stops the shooter motor
    public void stopShooter() {
        targetVelocity = 0.0;
        shooterMotor.set(0);
    }

}
