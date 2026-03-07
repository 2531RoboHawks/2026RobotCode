package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake;

/** Pivots the intake up. Use: "IntakeUp" in PathPlanner. */
public class IntakeUpCommand extends InstantCommand {
    public IntakeUpCommand(intake intakeSubsystem) {
        super(() -> intakeSubsystem.pivotToUp(), intakeSubsystem);
    }
}
