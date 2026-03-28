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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    //private final Telemetry logger=new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
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

    private final GenericEntry slowModeEntry;
    private final GenericEntry headingEntry;
    private final GenericEntry readyToShootEntry;
    private final GenericEntry hubActiveEntry;
    private final GenericEntry timeToShiftEntry;
    private final GenericEntry currentShiftEntry;

    private volatile boolean readyToShoot = false;

    private static final double HOOD_MANUAL_SPEED = 0.3;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        //drivetrain.registerTelemetry(logger::telemeterize);
        DataLogManager.start();
        debugSubsystem = new Debug(drivetrain);

        // ── Named Commands ────────────────────────────────────────────────────
        //
        // Command         | What It Does                                      | Subsystems                                        | Timeout
        // ----------------|---------------------------------------------------|---------------------------------------------------|---------------------------
        // IntakeDown      | Pivots intake arm down (instant)                  | intake                                            | Instant
        // IntakeUp        | Pivots intake arm up (instant)                    | intake                                            | Instant
        // RunIntake       | Runs intake roller motors                         | intake                                            | 5.5s
        // SpinShooter     | Spins shooter wheel to speed                      | shooter                                           | 1.5s
        // FeedBall        | Runs sorter + feeder                              | sorter, ShooterFeeder                             | 1.0s
        // Shoot           | Spin up shooter, then feed (shooter stays on)     | shooter, sorter, ShooterFeeder                    | 1.5s spin-up + 7.0s feed
        // QuickShoot      | Feed + shoot simultaneously (no spin-up delay)    | shooter, sorter, ShooterFeeder                    | 7.0s
        // AutoAlign       | Rotate to face target + adjust hood               | drivetrain, Hoodsubsystem                         | 2.5s
        // AlignAndShoot   | Align + spin up in parallel, then feed            | drivetrain, Hoodsubsystem, shooter, sorter, feeder| 2.0s align + 4.0s feed
        // Wait1s-4s       | Wait 1-4 seconds                                 | none                                              | 1-4s
        //

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

        // Pop shot — close-range quick shot, hood stays flat (stowed), no alignment
        NamedCommands.registerCommand("PopShot",
            new SequentialCommandGroup(
                new InstantCommand(() -> Hoodsubsystem.stow(), Hoodsubsystem),
                new ParallelDeadlineGroup(
                    new FeedBallCommand(sorterSubsystem, feederSubsystem)
                        .withTimeout(Constants.Timeouts.kFeedBallTimeout),
                    new SpinShooterCommand(shooterSubsystem)
                ),
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)
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
        slowModeEntry       = tab.add("Slow Mode Active", false).getEntry();
        headingEntry        = tab.add("Robot Heading (deg)", 0).getEntry();
        readyToShootEntry   = tab.add("Ready to Shoot", false).getEntry();
        hubActiveEntry      = tab.add("Hub Active", false).getEntry();
        timeToShiftEntry    = tab.add("Time to Shift", 0.0).getEntry();
        currentShiftEntry   = tab.add("Current Shift", "---").getEntry();

        // Vision thread for ready-to-shoot image indicator
        Thread readyImageThread = new Thread(() -> {
            try {
                System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME);
                String deployDir = Filesystem.getDeployDirectory().getAbsolutePath();
                System.out.println("[ReadyImage] Loading from: " + deployDir);

                Mat trueImage  = Imgcodecs.imread(deployDir + "/true.jpg");
                Mat falseImage = Imgcodecs.imread(deployDir + "/false.jpg");
                System.out.println("[ReadyImage] true.jpg: " + (trueImage.empty() ? "MISSING" : trueImage.width() + "x" + trueImage.height()));
                System.out.println("[ReadyImage] false.jpg: " + (falseImage.empty() ? "MISSING" : falseImage.width() + "x" + falseImage.height()));

                int width  = falseImage.empty() ? 320 : falseImage.width();
                int height = falseImage.empty() ? 240 : falseImage.height();
                CvSource output = CameraServer.putVideo("Ready to Shoot", width, height);

                while (!Thread.interrupted()) {
                    output.putFrame(readyToShoot ? trueImage : falseImage);
                    try { Thread.sleep(100); } catch (InterruptedException e) { break; }
                }
            } catch (Exception | UnsatisfiedLinkError e) {
                System.err.println("[ReadyImage] Thread crashed: " + e.getMessage());
                e.printStackTrace();
            }
        });
        readyImageThread.setDaemon(true);
        readyImageThread.start();

        configureBindings();

        // Keeps Hoodsubsystem registered with the scheduler so periodic() always runs
        Hoodsubsystem.setDefaultCommand(new RunCommand(() -> {}, Hoodsubsystem));
    }

    // ── Right bumper: full shoot pipeline ─────────────────────────────────────
    // If close enough, does a pop shot (hood flat, no align). Otherwise full align + shoot.
    // Feed stage has NO timeout — runs until driver releases right bumper
    private Command buildShootPipeline() {
        return new ConditionalCommand(
            // Close range: pop shot — hood flat, skip alignment, just spin + feed
            buildPopShotPipeline(),
            // Far range: full align + shoot
            buildAlignShootPipeline(),
            // Condition: is the robot close to the target?
            () -> {
                double dist = AutoAlignCommand.getDistanceToTarget();
                return dist > 0 && dist < Constants.AutoAlign.kPopShotDistance;
            }
        );
    }

    private Command buildPopShotPipeline() {
        return new SequentialCommandGroup(
            // Phase 1: auto-align (with hood from distance table) + spin up pop shot velocity
            new ParallelDeadlineGroup(
                new AutoAlignCommand(drivetrain, Hoodsubsystem)
                    .withTimeout(Constants.Timeouts.kShootPipelineAlignTimeout),
                new RunCommand(() -> {
                    shooterSubsystem.runPopShotMotor();
                    candleSubsystem.setState(CandleState.ALIGNING);
                }, shooterSubsystem)
            ),
            // Phase 2: feed while holding brake — runs until button released
            new ParallelDeadlineGroup(
                new StartEndCommand(
                    () -> {
                        sorterSubsystem.runSorterMotor();
                        feederSubsystem.runFeederMotor();
                        shooterSubsystem.runPopShotMotor();
                        candleSubsystem.setState(CandleState.SHOOTING);
                        noBallTimer.reset();
                        noBallTimer.start();
                        feederWasRunning = true;
                    },
                    () -> {
                        sorterSubsystem.stop();
                        feederSubsystem.stop();
                        shooterSubsystem.stopShooter();
                        Hoodsubsystem.stow();
                        noBallTimer.stop();
                        feederWasRunning = false;
                        candleSubsystem.setState(CandleState.DEFAULT);
                    },
                    sorterSubsystem, feederSubsystem, shooterSubsystem
                ),
                new RunCommand(() -> drivetrain.setControl(brake), drivetrain)
            )
        );
    }

    private Command buildAlignShootPipeline() {
        return new SequentialCommandGroup(
            // Phase 1: align + spin up shooter in parallel (long-range if >4m)
            new ParallelDeadlineGroup(
                new AutoAlignCommand(drivetrain, Hoodsubsystem)
                    .withTimeout(Constants.Timeouts.kShootPipelineAlignTimeout),
                new RunCommand(() -> {
                    double dist = AutoAlignCommand.getDistanceToTarget();
                    if (dist > Constants.Shooter.kLongRangeDistance) {
                        shooterSubsystem.runLongRangeMotor();
                    } else {
                        shooterSubsystem.runShooterMotor();
                    }
                    candleSubsystem.setState(CandleState.ALIGNING);
                }, shooterSubsystem)
            ),
            // Phase 2: feed + hold drivetrain in brake — no timeout, runs until button released
            new ParallelDeadlineGroup(
                new StartEndCommand(
                    () -> {
                        sorterSubsystem.runSorterMotor();
                        feederSubsystem.runFeederMotor();
                        double dist = AutoAlignCommand.getDistanceToTarget();
                        if (dist > Constants.Shooter.kLongRangeDistance) {
                            shooterSubsystem.runLongRangeMotor();
                        } else {
                            shooterSubsystem.runShooterMotor();
                        }
                        candleSubsystem.setState(CandleState.SHOOTING);
                        noBallTimer.reset();
                        noBallTimer.start();
                        feederWasRunning = true;
                    },
                    () -> {
                        sorterSubsystem.stop();
                        feederSubsystem.stop();
                        shooterSubsystem.stopShooter();
                        Hoodsubsystem.stow();
                        noBallTimer.stop();
                        feederWasRunning = false;
                        candleSubsystem.setState(CandleState.DEFAULT);
                    },
                    sorterSubsystem, feederSubsystem, shooterSubsystem
                ),
                new RunCommand(() -> drivetrain.setControl(brake), drivetrain)
            )
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
                intakeSubsystem.lockPositionWithNudge(Constants.Intake.kRollerDownNudge);
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

        driverController.x().whileTrue(
            new StartEndCommand(
                () -> intakeSubsystem.pivotToDown(),
                () -> intakeSubsystem.stopPivot(),
                intakeSubsystem
            )
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

        // Right trigger: capture auto-align tuning data snapshot
        secondController.rightTrigger().onTrue(
            new InstantCommand(() -> Hoodsubsystem.captureAutoAlignData())
        );

        // Left bumper: hood homing (pits only — press once, runs automatically)
        secondController.leftBumper().onTrue(Hoodsubsystem.homeCommand());
    }

    // ── Hub shift tracking ─────────────────────────────────────────────────
    // Shift boundaries (matchTime counts DOWN): 130, 105, 80, 55, 30
    private static final double[] SHIFT_BOUNDARIES = {130, 105, 80, 55, 30};
    private static final String[] SHIFT_NAMES = {"Transition", "Shift 1", "Shift 2", "Shift 3", "Shift 4", "Endgame"};

    private boolean isHubActive() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default  -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red  -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) return true;             // Transition
        else if (matchTime > 105) return shift1Active; // Shift 1
        else if (matchTime > 80) return !shift1Active; // Shift 2
        else if (matchTime > 55) return shift1Active;  // Shift 3
        else if (matchTime > 30) return !shift1Active; // Shift 4
        else return true;                              // Endgame
    }

    private String getCurrentShiftName() {
        if (!DriverStation.isTeleopEnabled()) return "---";
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > 130) return SHIFT_NAMES[0];
        else if (matchTime > 105) return SHIFT_NAMES[1];
        else if (matchTime > 80) return SHIFT_NAMES[2];
        else if (matchTime > 55) return SHIFT_NAMES[3];
        else if (matchTime > 30) return SHIFT_NAMES[4];
        else return SHIFT_NAMES[5];
    }

    private double getTimeToNextShift() {
        if (!DriverStation.isTeleopEnabled()) return 0;
        double matchTime = DriverStation.getMatchTime();
        for (double boundary : SHIFT_BOUNDARIES) {
            if (matchTime > boundary) {
                return matchTime - boundary;
            }
        }
        return matchTime; // In endgame, counts down to 0
    }

    public void periodic() {
        slowModeEntry.setBoolean(driverController.leftTrigger().getAsBoolean());
        headingEntry.setDouble(drivetrain.getState().Pose.getRotation().getDegrees());

        boolean limelightHasTarget = NetworkTableInstance.getDefault()
            .getTable(Constants.AutoAlign.kLimelightName)
            .getEntry("tv")
            .getDouble(0) == 1.0;

        readyToShoot = shooterSubsystem.isAtSpeed()
            && limelightHasTarget
            && Hoodsubsystem.isAtGoal();
        readyToShootEntry.setBoolean(readyToShoot);

        // Hub shift info
        hubActiveEntry.setBoolean(isHubActive());
        timeToShiftEntry.setDouble(getTimeToNextShift());
        currentShiftEntry.setString(getCurrentShiftName());

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