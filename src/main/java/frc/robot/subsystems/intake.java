package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {

    private static final int PIVOT_MOTOR_ID = 22;
    private static final int ROLLER_MOTOR_ID = 31;

    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
    private final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID);

    private static final double UP_POSITION = 0;
    private static final double DOWN_POSITION = 2;

    private final MotionMagicDutyCycle motionMagic =
        new MotionMagicDutyCycle(0).withSlot(0);

    public intake() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotionMagic.MotionMagicCruiseVelocity = 20;
        config.MotionMagic.MotionMagicAcceleration = 40;

        config.Slot0.kP = 0.25;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        pivotMotor.getConfigurator().apply(config);

        pivotMotor.setPosition(0);
    }

    // Y button — drives to UP position, then re-zeros once it arrives
    public Command pivotUp() {
        return run(() ->
            pivotMotor.setControl(motionMagic.withPosition(UP_POSITION))
        ).until(() ->
            Math.abs(pivotMotor.getPosition().getValueAsDouble()) < 0.3
        ).andThen(runOnce(() ->
            pivotMotor.setPosition(0)
        ));
    }

    // X button — go to DOWN position
    public Command pivotDown() {
        return runOnce(() ->
            pivotMotor.setControl(motionMagic.withPosition(DOWN_POSITION))
        );
    }

    // Left trigger — spin roller while held, stop on release
    public Command runRoller() {
        return runEnd(
            () -> rollerMotor.set(-1.0),
            () -> rollerMotor.set(0.0)
        );
    }
}