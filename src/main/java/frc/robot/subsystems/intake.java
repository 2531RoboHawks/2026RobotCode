package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {

    // 🔴 CHANGE THESE TO YOUR REAL CAN IDS
    private static final int PIVOT_MOTOR_ID = 22;
    private static final int ROLLER_MOTOR_ID = 31;

    private TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID);

    // SETPOINTS
    // ZERO = INTAKE UP
    private static final double PIVOT_UP_POSITION = 0.0;
    private static final double PIVOT_DOWN_POSITION = 5.0; // 🔧 tune this later

    private PositionDutyCycle positionRequest = new PositionDutyCycle(0);

    // 🔴 CONSTRUCTOR
    public intake() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID
        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;

        // 🛑 SOFT LIMITS (UP = 0)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.0;
        // cannot go too far DOWN

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.2;
        // cannot go above UP

        // 🛡️ CURRENT LIMIT
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 35;

        pivotMotor.getConfigurator().apply(config);

        // Flip if it moves wrong direction

        //IMPORTANT CHECK CHAT FOR THIS

        // ZERO encoder at startup
        pivotMotor.setPosition(0);
    }

    // 🔵 GO TO UP POSITION
    public Command pivotUp() {
        return runOnce(() ->
            pivotMotor.setControl(positionRequest.withPosition(PIVOT_UP_POSITION))
        );
    }

    // 🔵 GO TO DOWN POSITION
    public Command pivotDown() {
        return runOnce(() ->
            pivotMotor.setControl(positionRequest.withPosition(PIVOT_DOWN_POSITION))
        );
    }

    // 🟢 ROLLER RUNS WHILE HELD
    public Command runRoller() {
        return runEnd(
            () -> rollerMotor.set(-1.0),
            () -> rollerMotor.set(0.0)
        );
    }

    @Override
    public void periodic() {
        // 🔬 Used to find your real DOWN value
        System.out.println(pivotMotor.getPosition().getValueAsDouble());
    }
}