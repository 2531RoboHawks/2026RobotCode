package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(40);

    public void runFeederMotor() {
        feederMotor.set(-0.3);
    }

    public void runFeederMotorReverse() {
        feederMotor.set(0.8);
    }

    public void stop() {
        feederMotor.set(0);
    }
}