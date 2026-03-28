package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/** Pivots the intake down. Use: "IntakeDown" in PathPlanner. */
public class IntakeDownCommand extends InstantCommand {
    public IntakeDownCommand(Intake intakeSubsystem) {
        super(() -> intakeSubsystem.pivotToDown(), intakeSubsystem);
    }
}
