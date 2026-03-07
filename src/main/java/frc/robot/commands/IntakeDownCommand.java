package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake;

/** Pivots the intake down. Use: "IntakeDown" in PathPlanner. */
public class IntakeDownCommand extends InstantCommand {
    public IntakeDownCommand(intake intakeSubsystem) {
        super(() -> intakeSubsystem.pivotToDown(), intakeSubsystem);
    }
}
