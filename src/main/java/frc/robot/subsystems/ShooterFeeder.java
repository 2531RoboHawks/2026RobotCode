package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(40);

    // Runs the feeder motor at -30% to push game pieces toward the shooter
    // NOTE: not fully certain if -0.3 is toward shooter or away, depends on motor orientation
    public void runFeederMotor() {
        feederMotor.set(-0.3);
    }

    // Runs the feeder motor in reverse at 80% to eject/unjam game pieces
    public void runFeederMotorReverse() {
        feederMotor.set(0.8);
    }

    // Stops the feeder motor
    public void stop() {
        feederMotor.set(0);
    }
}