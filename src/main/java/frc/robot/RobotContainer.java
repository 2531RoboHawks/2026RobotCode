package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.subsystems.Hoodsubsystem;
import frc.robot.subsystems.Debug;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final intake intakeSubsystem          = new intake();
    private final sorter sorterSubsystem          = new sorter();
    private final ShooterFeeder feederSubsystem   = new ShooterFeeder();
    private final shooter shooterSubsystem        = new shooter();
    private final Hoodsubsystem Hoodsubsystem     = new Hoodsubsystem();
    private final CandleSubsystem candleSubsystem = new CandleSubsystem();

    private final edu.wpi.first.wpilibj.Timer noBallTimer = new edu.wpi.first.wpilibj.Timer();
    private boolean feederWasRunning = false;

    private double MaxSpeed       = Constants.Swerve.kMaxSpeedMultiplier * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRate).in(RadiansPerSecond);

    private final CommandXboxController driverController = new CommandXboxController(Constants.OI.kDriverControllerPort);
    private final CommandXboxController secondController = new CommandXboxController(Constants.OI.kSecondControllerPort);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandPercent)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandPercent)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandSwerveDrivetrain drivetrain;
    @SuppressWarnings("unused")
    private Debug debugSubsystem;

    private final GenericEntry batteryVoltageEntry;
    private final GenericEntry slowModeEntry;
    private final GenericEntry headingEntry;
    private final GenericEntry readyToShootEntry;

    private static final double HOOD_MANUAL_SPEED = 0.3;

    public RobotContainer() {

        drivetrain = TunerConstants.createDrivetrain();
        debugSubsystem = new Debug(drivetrain);

        // ── Named Commands ────────────────────────────────────────────────────

        NamedCommands.registerCommand("IntakeDown",
            new IntakeDownCommand(intakeSubsystem));

        NamedCommands.registerCommand("IntakeUp",
            new IntakeUpCommand(intakeSubsystem));

        NamedCommands.registerCommand("RunIntake",
            new RunIntakeCommand(intakeSubsystem)
                .withTimeout(Constants.Timeouts.kRunIntakePhase2Timeout));

        NamedCommands.registerCommand("SpinShooter",
            new SpinShooterCommand(shooterSubsystem)
                .withTimeout(Constants.Timeouts.kSpinShooterTimeout));

        NamedCommands.registerCommand("FeedBall",
            new FeedBallCommand(sorterSubsystem, feederSubsystem)
                .withTimeout(Constants.Timeouts.kFeedBallTimeout));

        NamedCommands.registerCommand("Shoot",
            new SequentialCommandGroup(
                // Phase 1: spin up shooter
                new SpinShooterCommand(shooterSubsystem)
                    .withTimeout(Constants.Timeouts.kShootSpinUpTimeout),
                // Phase 2: feed while keeping shooter running
                new ParallelDeadlineGroup(
                    new FeedBallCommand(sorterSubsystem, feederSubsystem)
                        .withTimeout(Constants.Timeouts.kShootFeedTimeout),
                    new SpinShooterCommand(shooterSubsystem)
                )
            )
        );

        NamedCommands.registerCommand("QuickShoot",
            new ParallelDeadlineGroup(
                new FeedBallCommand(sorterSubsystem, feederSubsystem)
                    .withTimeout(Constants.Timeouts.kShootFeedTimeout),
                new SpinShooterCommand(shooterSubsystem)
            )
        );

        NamedCommands.registerCommand("AutoAlign",
            new AutoAlignCommand(drivetrain, Hoodsubsystem)
                .withTimeout(Constants.Timeouts.kAutoAlignTimeout));

        NamedCommands.registerCommand("AlignAndShoot",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new AutoAlignCommand(drivetrain, Hoodsubsystem)
                        .withTimeout(Constants.Timeouts.kShootPipelineAlignTimeout),
                    new SpinShooterCommand(shooterSubsystem)
                ),
                new StartEndCommand(
                    () -> {
                        sorterSubsystem.runSorterMotor();
                        feederSubsystem.runFeederMotor();
                        shooterSubsystem.runShooterMotor();
                    },
                    () -> {
                        sorterSubsystem.stop();
                        feederSubsystem.stop();
                        shooterSubsystem.stopShooter();
                        Hoodsubsystem.stow();
                    },
                    sorterSubsystem, feederSubsystem, shooterSubsystem, Hoodsubsystem
                ).withTimeout(Constants.Timeouts.kShootPipelineFeedTimeout)
            )
        );

        NamedCommands.registerCommand("Wait1s", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("Wait2s", Commands.waitSeconds(2.0));
        NamedCommands.registerCommand("Wait3s", Commands.waitSeconds(3.0));
        NamedCommands.registerCommand("Wait4s", Commands.waitSeconds(4.0));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field", drivetrain.getField());

        ShuffleboardTab tab = Shuffleboard.getTab("Important");
        batteryVoltageEntry = tab.add("Battery Voltage (V)", 0).getEntry();
        slowModeEntry       = tab.add("Slow Mode Active", false).getEntry();
        headingEntry        = tab.add("Robot Heading (deg)", 0).getEntry();
        readyToShootEntry   = tab.add("Ready to Shoot", false).getEntry();

        configureBindings();

        // Keeps Hoodsubsystem registered with the scheduler so periodic() always runs
        Hoodsubsystem.setDefaultCommand(new RunCommand(() -> {}, Hoodsubsystem));
    }

    // ── Right bumper: full shoot pipeline ─────────────────────────────────────
    // Feed stage has NO timeout — runs until driver releases right bumper
    private Command buildShootPipeline() {
        return new SequentialCommandGroup(
            // Phase 1: align + spin up shooter in parallel
            new ParallelDeadlineGroup(
                new AutoAlignCommand(drivetrain, Hoodsubsystem)
                    .withTimeout(Constants.Timeouts.kShootPipelineAlignTimeout),
                new StartEndCommand(
                    () -> {
                        shooterSubsystem.runShooterMotor();
                        candleSubsystem.setState(CandleState.ALIGNING);
                    },
                    () -> {}, // shooter intentionally kept running into phase 2
                    shooterSubsystem // ← candleSubsystem removed, no requirement conflict
                )
            ),
            // Phase 2: feed — no timeout, runs until button released
            new StartEndCommand(
                () -> {
                    sorterSubsystem.runSorterMotor();
                    feederSubsystem.runFeederMotor();
                    shooterSubsystem.runShooterMotor();
                    candleSubsystem.setState(CandleState.SHOOTING);
                    noBallTimer.reset();
                    noBallTimer.start();
                    feederWasRunning = true;
                },
                () -> {
                    sorterSubsystem.stop();
                    feederSubsystem.stop();
                    shooterSubsystem.stopShooter(); // ← guaranteed to run on release
                    Hoodsubsystem.stow();
                    noBallTimer.stop();
                    feederWasRunning = false;
                    candleSubsystem.setState(CandleState.DEFAULT);
                },
                sorterSubsystem, feederSubsystem, shooterSubsystem // hood not required
            ) // ← no .withTimeout() — button release cancels via whileTrue
        );
    }

    // ── Second controller A-button: align only (no shoot) ────────────────────
    private Command buildAlignOnly() {
        return new AutoAlignCommand(drivetrain, Hoodsubsystem)
            .withTimeout(Constants.Timeouts.kShootPipelineAlignTimeout);
    }

    private void configureBindings() {

        // ── DRIVER CONTROLLER ─────────────────────────────────────────────────

        // Right bumper: full auto shoot pipeline (no feed timeout)
        driverController.rightBumper().whileTrue(buildShootPipeline());

        // Left bumper: intake lock
        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> intakeSubsystem.lockPosition(), intakeSubsystem))
            .onFalse(new InstantCommand(() -> intakeSubsystem.unlockPosition(), intakeSubsystem));

        // Left trigger: slow mode + run rollers + lock arm position
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double speedMult = driverController.leftTrigger().getAsBoolean()
                    ? Constants.Swerve.kSlowModeMultiplier : 1.0;
                return drive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMult)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMult)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMult);
            })
        );
        driverController.leftTrigger().whileTrue(
            new RunCommand(() -> {
                intakeSubsystem.runRollerMotor();
                intakeSubsystem.lockPosition();
            }, intakeSubsystem)
        );
        driverController.leftTrigger().onFalse(
            new InstantCommand(() -> {
                intakeSubsystem.stopRoller();
                intakeSubsystem.unlockPosition();
            }, intakeSubsystem)
        );

        // Right trigger: manual shooter spin
        driverController.rightTrigger().whileTrue(
            new RunCommand(() -> {
                shooterSubsystem.runShooterMotor();
                candleSubsystem.setState(CandleState.ALIGNING);
            }, shooterSubsystem, candleSubsystem)
        );
        driverController.rightTrigger().onFalse(
            new InstantCommand(() -> {
                shooterSubsystem.stopShooter();
                candleSubsystem.setState(CandleState.DEFAULT);
            }, shooterSubsystem)
        );

        // DPad left: manual shoot sequence (no alignment)
        driverController.povLeft().whileTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    shooterSubsystem.runShooterMotor();
                    candleSubsystem.setState(CandleState.ALIGNING);
                }, shooterSubsystem)
                    .withTimeout(Constants.Timeouts.kManualShootSpinUpTimeout),
                new RunCommand(() -> {
                    sorterSubsystem.runSorterMotor();
                    feederSubsystem.runFeederMotor();
                    shooterSubsystem.runShooterMotor();
                    candleSubsystem.setState(CandleState.SHOOTING);
                }, sorterSubsystem, feederSubsystem, shooterSubsystem)
            )
        );
        driverController.povLeft().onFalse(
            new InstantCommand(() -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
                shooterSubsystem.stopShooter();
                candleSubsystem.setState(CandleState.DEFAULT);
            }, sorterSubsystem, feederSubsystem, shooterSubsystem)
        );

        // DPad up: manual sorter + feeder
        driverController.povUp().whileTrue(
            new RunCommand(() -> {
                sorterSubsystem.runSorterMotor();
                feederSubsystem.runFeederMotor();
                candleSubsystem.setState(CandleState.SHOOTING);
            }, sorterSubsystem, feederSubsystem)
        );
        driverController.povUp().onFalse(
            new InstantCommand(() -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
                candleSubsystem.setState(CandleState.DEFAULT);
            }, sorterSubsystem, feederSubsystem)
        );

        // ── SECOND CONTROLLER ─────────────────────────────────────────────────

        // A button: align + hood only, no shoot
        secondController.a().whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> candleSubsystem.setState(CandleState.ALIGNING)),
                buildAlignOnly()
            )
        );
        secondController.a().onFalse(
            new InstantCommand(() -> {
                Hoodsubsystem.stow();
                candleSubsystem.setState(CandleState.DEFAULT);
            }, Hoodsubsystem)
        );

        // DPad up: manually move hood flap up
        secondController.povUp().whileTrue(
            new RunCommand(() ->
                Hoodsubsystem.setPosition(
                    Hoodsubsystem.getTargetPosition() + HOOD_MANUAL_SPEED * 0.02
                ),
                Hoodsubsystem
            )
        );

        // DPad down: manually move hood flap down
        secondController.povDown().whileTrue(
            new RunCommand(() ->
                Hoodsubsystem.setPosition(
                    Hoodsubsystem.getTargetPosition() - HOOD_MANUAL_SPEED * 0.02
                ),
                Hoodsubsystem
            )
        );

        // Y button: pivot intake up
        secondController.y().whileTrue(
            new StartEndCommand(
                () -> intakeSubsystem.pivotToUp(),
                () -> intakeSubsystem.stopPivot(),
                intakeSubsystem
            )
        );

        // X button: pivot intake down
        secondController.x().whileTrue(
            new StartEndCommand(
                () -> intakeSubsystem.pivotToDown(),
                () -> intakeSubsystem.stopPivot(),
                intakeSubsystem
            )
        );

        // B button: reverse feeder + sorter
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
    }

    public void periodic() {
        batteryVoltageEntry.setDouble(RobotController.getBatteryVoltage());
        slowModeEntry.setBoolean(driverController.leftTrigger().getAsBoolean());
        headingEntry.setDouble(drivetrain.getState().Pose.getRotation().getDegrees());

        boolean limelightHasTarget = NetworkTableInstance.getDefault()
            .getTable(Constants.AutoAlign.kLimelightName)
            .getEntry("tv")
            .getDouble(0) == 1.0;

        readyToShootEntry.setBoolean(
            shooterSubsystem.isAtSpeed()
            && limelightHasTarget
            && Hoodsubsystem.isAtGoal()
        );

        // No-ball detection: feeder running but no load for 0.5s → blink red
        if (feederWasRunning) {
            if (feederSubsystem.isUnderLoad()) {
                noBallTimer.reset();
            } else if (noBallTimer.hasElapsed(0.5)) {
                candleSubsystem.setState(CandleState.NO_BALL);
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}