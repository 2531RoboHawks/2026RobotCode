package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.kShooterMotorID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Shooter.kOpenLoopRampPeriod;

        // Velocity PID — tune kP if wheel bogs down under load
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = Constants.Shooter.kP;
        slot0.kV = Constants.Shooter.kV;

        shooterMotor.getConfigurator().apply(config);
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
        shooterMotor.setControl(velocityRequest.withVelocity(Constants.Shooter.kShooterVelocity));
    }

    // Stops the shooter motor
    public void stopShooter() {
        shooterMotor.set(0);
    }

}
