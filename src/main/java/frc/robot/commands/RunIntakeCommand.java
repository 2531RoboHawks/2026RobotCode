package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake;

/**
 * Runs the intake rollers until the command ends or times out.
 * Use: "RunIntake" in PathPlanner (add a timeout in the GUI, e.g. 2s).
 */
public class RunIntakeCommand extends StartEndCommand {
    public RunIntakeCommand(intake intakeSubsystem) {
        super(
            () -> intakeSubsystem.runRollerMotor(),
            () -> intakeSubsystem.stopRoller(),
            intakeSubsystem
        );
    }
}
