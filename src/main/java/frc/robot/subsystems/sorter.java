package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sorter extends SubsystemBase {
    private final TalonFX sorterMotor = new TalonFX(Constants.Sorter.kSorterMotorID);

    // Runs the sorter motor to move game pieces toward the feeder/shooter
    public void runSorterMotor() {
        sorterMotor.set(Constants.Sorter.kForwardSpeed);
    }

    // Runs the sorter motor in reverse to eject/unjam game pieces
    public void runSorterMotorReverse() {
        sorterMotor.set(Constants.Sorter.kReverseSpeed);
    }

    // Stops the sorter motor
    public void stop() {
        sorterMotor.set(0);
    }
}
