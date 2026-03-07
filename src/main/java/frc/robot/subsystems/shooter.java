package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {

    private static final int SHOOTER_MOTOR_ID = 24;

    private final TalonFX shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 2.0;

        // Velocity PID — tune kP if wheel bogs down under load
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.55;  // increase if wheel slows down when ball hits
        slot0.kV = 0.10; // feedforward — helps reach target speed faster

        shooterMotor.getConfigurator().apply(config);
    }

    public Command runShooter() {
        return runEnd(
            () -> runShooterMotor(),
            () -> stopShooter()
        );
    }

    public void runShooterMotor() {
        shooterMotor.setControl(velocityRequest.withVelocity(54)); // rotations per second, tune this
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

}