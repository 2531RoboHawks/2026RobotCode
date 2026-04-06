package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.Feeder.kFeederMotorID);

    public ShooterFeeder() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit       = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        feederMotor.getConfigurator().apply(config);
        feederMotor.getPosition().setUpdateFrequency(10);
        feederMotor.getFault_Hardware().setUpdateFrequency(4);
    }

    public void runFeederMotor() {
        feederMotor.set(Constants.Feeder.kForwardSpeed);
    }

    public void runFeederMotorReverse() {
        feederMotor.set(Constants.Feeder.kReverseSpeed);
    }

    public void stop() {
        feederMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Stator Current", feederMotor.getStatorCurrent().getValueAsDouble());
    }

    // Returns true if a ball is currently passing through (motor under load)
    public boolean isUnderLoad() {
        return feederMotor.getStatorCurrent().getValueAsDouble()
               > Constants.Feeder.kBallDetectCurrentThreshold;
    }
}