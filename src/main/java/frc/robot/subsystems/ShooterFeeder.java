package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.Feeder.kFeederMotorID);

    // Runs the feeder motor to push game pieces toward the shooter
    public void runFeederMotor() {
        feederMotor.set(Constants.Feeder.kForwardSpeed);
    }

    // Runs the feeder motor in reverse to eject/unjam game pieces
    public void runFeederMotorReverse() {
        feederMotor.set(Constants.Feeder.kReverseSpeed);
    }

    // Stops the feeder motor
    public void stop() {
        feederMotor.set(0);
    }
}
