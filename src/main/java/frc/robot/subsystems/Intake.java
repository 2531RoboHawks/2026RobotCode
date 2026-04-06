package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(Constants.Intake.kPivotMotorID);
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.kRollerMotorID);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private final GenericEntry pivotPositionEntry;
    private final GenericEntry rollerCurrentEntry;

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID for holding position — increase kP if it's not holding strong enough
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = Constants.Intake.kPivotP;
        slot0.kD = Constants.Intake.kPivotD;

        config.CurrentLimits.StatorCurrentLimit       = Constants.Intake.kPivotHoldCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
        pivotMotor.getFault_Hardware().setUpdateFrequency(4);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.StatorCurrentLimit       = Constants.Intake.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.getPosition().setUpdateFrequency(10);
        rollerMotor.getVelocity().setUpdateFrequency(10);
        rollerMotor.getFault_Hardware().setUpdateFrequency(4);

        ShuffleboardTab tab = Shuffleboard.getTab("Important");
        pivotPositionEntry = tab.add("Pivot Position", 0).getEntry();
        rollerCurrentEntry = tab.add("Roller Current (A)", 0).getEntry();
    }

    @Override
    public void periodic() {
        pivotPositionEntry.setDouble(getPivotPosition());
        rollerCurrentEntry.setDouble(rollerMotor.getSupplyCurrent().getValueAsDouble());
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

    // Locks the pivot arm pushed slightly downward (positive offset = down)
    public void lockPositionWithNudge(double offsetRotations) {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        pivotMotor.setControl(positionRequest.withPosition(currentPosition + offsetRotations));
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
