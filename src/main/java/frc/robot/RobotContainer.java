package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.FeedBallCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.sorter;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.shooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final intake intakeSubsystem = new intake();
    private final sorter sorterSubsystem = new sorter();
    private final ShooterFeeder feederSubsystem = new ShooterFeeder();
    private final shooter shooterSubsystem = new shooter();

    private double MaxSpeed = Constants.Swerve.kMaxSpeedMultiplier * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRate).in(RadiansPerSecond);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController secondController = new CommandXboxController(1);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandPercent)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandPercent)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric limelightDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandPercent)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandSwerveDrivetrain drivetrain;

public RobotContainer() {

    // ✅ Initialize drivetrain FIRST
    drivetrain = TunerConstants.createDrivetrain();

    // Then register named commands
NamedCommands.registerCommand("IntakeDownSai",
    new StartEndCommand(
        () -> intakeSubsystem.pivotToDown(),
        () -> intakeSubsystem.stopPivot(),
        intakeSubsystem
    ).withTimeout(0.2)
);



NamedCommands.registerCommand("IntakeUpSai",
    new StartEndCommand(
        () -> intakeSubsystem.pivotToUp(),
        () -> intakeSubsystem.stopPivot(),
        intakeSubsystem
    ).withTimeout(0.75)
);

NamedCommands.registerCommand("RunIntakeSai",
    new SequentialCommandGroup(
        new StartEndCommand(
            () -> intakeSubsystem.runRollerMotor(),
            () -> {},
            intakeSubsystem
        ).withTimeout(0.5), // rollers spin for 0.5s first
        new StartEndCommand(
            () -> {
                intakeSubsystem.lockPosition();
                intakeSubsystem.runRollerMotor();
            },
            () -> {
                intakeSubsystem.unlockPosition();
                intakeSubsystem.stopRoller();
            },
            intakeSubsystem
        ).withTimeout(5.5) // then lock + keep spinning for remaining time
    )
);

NamedCommands.registerCommand("SpinShooterSai",
    new SpinShooterCommand(shooterSubsystem).withTimeout(1.5));

NamedCommands.registerCommand("FeedBallSai",
    new FeedBallCommand(sorterSubsystem, feederSubsystem).withTimeout(1.0));

NamedCommands.registerCommand("ShootSai",
    new SequentialCommandGroup(
        new StartEndCommand(
            () -> shooterSubsystem.runShooterMotor(),
            () -> {},  // don't stop yet, keep spinning into feed phase
            shooterSubsystem
        ).withTimeout(1.5),
        new StartEndCommand(
            () -> {
                sorterSubsystem.runSorterMotor();
                feederSubsystem.runFeederMotor();
                shooterSubsystem.runShooterMotor();
            },
            () -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
                shooterSubsystem.stopShooter(); // ← stops here after feed is done
            },
            sorterSubsystem, feederSubsystem, shooterSubsystem
        ).withTimeout(7)
    )
);

NamedCommands.registerCommand("AutoAlignSai",
    new AutoAlignCommand(drivetrain).withTimeout(2.5));

NamedCommands.registerCommand("Wait1sSai", Commands.waitSeconds(1.0));
NamedCommands.registerCommand("Wait2sSai", Commands.waitSeconds(2.0));
NamedCommands.registerCommand("Wait3sSai", Commands.waitSeconds(3.0));
NamedCommands.registerCommand("Wait4sSai", Commands.waitSeconds(4.0));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field", drivetrain.getField());

    configureBindings();
}

    // Maps controller buttons/triggers to robot actions (drive, intake, shooter, etc.)
    private void configureBindings() {


driverController.rightBumper().whileTrue(
    drivetrain.applyRequest(() -> {
        int tagID = (int) NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tid")
            .getDouble(-1);

        if (!AutoAlignCommand.TAG_TARGETS.containsKey(tagID)) {
            return brake;
        }

        return limelightDrive
            .withVelocityX((LimelightHelpers.getTY("limelight") - AutoAlignCommand.getTargetTYStatic(tagID)) * -0.15)
            .withVelocityY((LimelightHelpers.getTX("limelight") - AutoAlignCommand.getTargetTXStatic(tagID)) * -0.15)
            .withRotationalRate(0);
    })
);

        driverController.leftBumper()
        .onTrue(new InstantCommand(() -> intakeSubsystem.lockPosition(), intakeSubsystem))
        .onFalse(new InstantCommand(() -> intakeSubsystem.unlockPosition(), intakeSubsystem));

        // Y BUTTON = PIVOT UP
        secondController.y().whileTrue(
            new StartEndCommand(
                () -> intakeSubsystem.pivotToUp(),
                () -> intakeSubsystem.stopPivot(),
                intakeSubsystem
            )
        );

        // X BUTTON = PIVOT DOWN
        secondController.x().whileTrue(
            new StartEndCommand(
                () -> intakeSubsystem.pivotToDown(),
                () -> intakeSubsystem.stopPivot(),
                intakeSubsystem
            )
        );

        // B BUTTON = REVERSE FEEDER + SORTER
        secondController.b().whileTrue(
            new RunCommand(() -> {
                sorterSubsystem.runSorterMotorReverse();
                feederSubsystem.runFeederMotorReverse();
            }, sorterSubsystem, feederSubsystem)
        );
        secondController.b().onFalse(
            new InstantCommand(() -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
            }, sorterSubsystem, feederSubsystem)
        );

// Default drive with slow mode on left trigger
drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> {
        double speedMult = driverController.leftTrigger().getAsBoolean() ? Constants.Swerve.kSlowModeMultiplier : 1.0;
        return drive
            .withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMult)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMult)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMult);
    })
);

// Left trigger also runs rollers (same button, both happen simultaneously)
driverController.leftTrigger().whileTrue(
    new RunCommand(() -> intakeSubsystem.runRollerMotor(), intakeSubsystem)
);
driverController.leftTrigger().onFalse(
    new InstantCommand(() -> intakeSubsystem.stopRoller(), intakeSubsystem)
);

        // RIGHT TRIGGER = SHOOTER
        driverController.rightTrigger().whileTrue(
            new RunCommand(() -> shooterSubsystem.runShooterMotor(), shooterSubsystem)
        );
        driverController.rightTrigger().onFalse(
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)
        );

        // DPAD LEFT = SHOOT SEQUENCE
        driverController.povLeft().whileTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> shooterSubsystem.runShooterMotor(), shooterSubsystem)
                    .withTimeout(1.5),
                new RunCommand(() -> {
                    sorterSubsystem.runSorterMotor();
                    feederSubsystem.runFeederMotor();
                    shooterSubsystem.runShooterMotor();
                }, sorterSubsystem, feederSubsystem, shooterSubsystem)
            )
        );
        driverController.povLeft().onFalse(
            new InstantCommand(() -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
                shooterSubsystem.stopShooter();
            }, sorterSubsystem, feederSubsystem, shooterSubsystem)
        );


        // DPAD UP = SORTER + FEEDER
        driverController.povUp().whileTrue(
            new RunCommand(() -> {
                sorterSubsystem.runSorterMotor();
                feederSubsystem.runFeederMotor();
            }, sorterSubsystem, feederSubsystem)
        );
        driverController.povUp().onFalse(
            new InstantCommand(() -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
            }, sorterSubsystem, feederSubsystem)
        );
    }

    // Returns the autonomous command selected from the SmartDashboard auto chooser dropdown
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}