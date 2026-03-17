package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Debug extends SubsystemBase {

    private static final String PREFIX = "debug/(debug) ";

    // Read-only motor references — Phoenix 6 allows multiple TalonFX objects on the same CAN ID
    private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.kShooterMotorID);
    private final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.kPivotMotorID);
    private final TalonFX intakeRollerMotor = new TalonFX(Constants.Intake.kRollerMotorID);
    private final TalonFX sorterMotor = new TalonFX(Constants.Sorter.kSorterMotorID);
    private final TalonFX feederMotor = new TalonFX(Constants.Feeder.kFeederMotorID);
    private final TalonFX hoodMotor = new TalonFX(Constants.Hood.kHoodMotorID, Constants.Hood.kCANbus);

    private final CommandSwerveDrivetrain drivetrain;

    public Debug(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // ── Motor temperatures (°C) ─────────────────────────────────────────
        SmartDashboard.putNumber(PREFIX + "Shooter Temp (C)",
                shooterMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Intake Pivot Temp (C)",
                intakePivotMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Intake Roller Temp (C)",
                intakeRollerMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Sorter Temp (C)",
                sorterMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Feeder Temp (C)",
                feederMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Hood Temp (C)",
                hoodMotor.getDeviceTemp().getValueAsDouble());

        // ── Swerve drive & steer motor temperatures ─────────────────────────
        for (int i = 0; i < 4; i++) {
            var module = drivetrain.getModule(i);
            SmartDashboard.putNumber(PREFIX + "Swerve " + i + " Drive Temp (C)",
                    module.getDriveMotor().getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(PREFIX + "Swerve " + i + " Steer Temp (C)",
                    module.getSteerMotor().getDeviceTemp().getValueAsDouble());
        }

        // ── Motor supply currents (A) ───────────────────────────────────────
        SmartDashboard.putNumber(PREFIX + "Shooter Current (A)",
                shooterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Intake Pivot Current (A)",
                intakePivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Intake Roller Current (A)",
                intakeRollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Sorter Current (A)",
                sorterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Feeder Current (A)",
                feederMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(PREFIX + "Hood Current (A)",
                hoodMotor.getSupplyCurrent().getValueAsDouble());

        // ── System-level debug ──────────────────────────────────────────────
        SmartDashboard.putNumber(PREFIX + "Battery Voltage (V)",
                RobotController.getBatteryVoltage());
        SmartDashboard.putNumber(PREFIX + "CAN Bus Utilization (%)",
                RobotController.getCANStatus().percentBusUtilization * 100.0);
        SmartDashboard.putBoolean(PREFIX + "Brownout",
                RobotController.isBrownedOut());
    }
}
