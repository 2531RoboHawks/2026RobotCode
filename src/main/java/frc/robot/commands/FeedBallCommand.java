package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.ShooterFeeder;

/**
 * Runs the sorter and feeder together until the command ends or times out.
 * Use: "FeedBall" in PathPlanner (add a timeout in the GUI, e.g. 1s).
 */
public class FeedBallCommand extends StartEndCommand {
    public FeedBallCommand(Sorter sorterSubsystem, ShooterFeeder feederSubsystem) {
        super(
            () -> {
                sorterSubsystem.runSorterMotor();
                feederSubsystem.runFeederMotor();
            },
            () -> {
                sorterSubsystem.stop();
                feederSubsystem.stop();
            },
            sorterSubsystem, feederSubsystem
        );
    }
}
