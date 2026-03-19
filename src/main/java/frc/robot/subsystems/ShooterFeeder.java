package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.Feeder.kFeederMotorID);

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