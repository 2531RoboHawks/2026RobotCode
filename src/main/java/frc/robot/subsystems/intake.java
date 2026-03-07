package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class intake extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(22);
    private final TalonFX rollerMotor = new TalonFX(31);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID for holding position — increase kP if it's not holding strong enough
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 4.0;  // ← increase this if still not holding
        slot0.kD = 0.1;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
    }

    public void pivotToUp() {
        pivotMotor.set(-0.15);
    }

    public void pivotToDown() {
        pivotMotor.set(0.15);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public void lockPosition() {
        // Grabs current position and actively holds it
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        pivotMotor.setControl(positionRequest.withPosition(currentPosition));
    }

    public void unlockPosition() {
        pivotMotor.set(0); // back to coast/free
    }

    public void runRollerMotor() {
        rollerMotor.set(-1);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
}