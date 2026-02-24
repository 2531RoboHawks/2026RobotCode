package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    private final intake intakeSubsystem = new intake();

    private double MaxSpeed =
        1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandXboxController driverController =
        new CommandXboxController(0);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick =
        new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    public RobotContainer() {

        // 🚀 GO TO SETPOINTS
        driverController.y().onTrue(intakeSubsystem.pivotUp());
        driverController.x().onTrue(intakeSubsystem.pivotDown());

        // 🟢 RUN ROLLER WHILE HELD
        driverController.leftTrigger().whileTrue(
            intakeSubsystem.runRoller()
        );

        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(
                        -joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle)
                .ignoringDisable(true)
        );

        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> brake));

        joystick.b().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(
                    new Rotation2d(
                        -joystick.getLeftY(),
                        -joystick.getLeftX()
                    )
                )
            )
        );

        joystick.back().and(joystick.y()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kForward));

        joystick.back().and(joystick.x()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.start().and(joystick.y()).whileTrue(
            drivetrain.sysIdQuasistatic(
                Direction.kForward));

        joystick.start().and(joystick.x()).whileTrue(
            drivetrain.sysIdQuasistatic(
                Direction.kReverse));

        joystick.leftBumper().onTrue(
            drivetrain.runOnce(
                drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(
            logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.applyRequest(() -> idle);
    }
}