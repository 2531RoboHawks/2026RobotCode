package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sorter extends SubsystemBase {

    private final TalonFX sorterMotor = new TalonFX(32);

    public void runSorterMotor() {
        sorterMotor.set(-0.7);
    }

    public void runSorterMotorReverse() {
        sorterMotor.set(0.7);
    }

    public void stop() {
        sorterMotor.set(0);
    }
}