package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class intake extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(Constants.Intake.kPivotMotorID);
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.kRollerMotorID);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID for holding position — increase kP if it's not holding strong enough
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = Constants.Intake.kPivotP;
        slot0.kD = Constants.Intake.kPivotD;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
    }

    // Runs the pivot motor to move the intake arm upward
    public void pivotToUp() {
        pivotMotor.set(Constants.Intake.kPivotUpSpeed);
    }

    // Runs the pivot motor to move the intake arm downward
    public void pivotToDown() {
        pivotMotor.set(Constants.Intake.kPivotDownSpeed);
    }

    // Stops the pivot motor (arm stays in place due to coast mode, may drift)
    public void stopPivot() {
        pivotMotor.set(0);
    }

    // Locks the pivot arm at its current position using closed-loop PID control
    public void lockPosition() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        pivotMotor.setControl(positionRequest.withPosition(currentPosition));
    }

    // Releases position hold, returning the pivot motor to open-loop (coast/free)
    public void unlockPosition() {
        pivotMotor.set(0);
    }

    // Spins the intake roller to pull game pieces in
    public void runRollerMotor() {
        rollerMotor.set(Constants.Intake.kRollerSpeed);
    }

    // Stops the intake roller motor
    public void stopRoller() {
        rollerMotor.set(0);
    }

    // Returns the current pivot motor encoder position (in rotations)
    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
}
