package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.shooter;

/**
 * Spins the shooter motors until the command ends or times out.
 * Use: "SpinShooter" in PathPlanner (add a timeout in the GUI, e.g. 1.5s).
 */
public class SpinShooterCommand extends StartEndCommand {
    public SpinShooterCommand(shooter shooterSubsystem) {
        super(
            () -> shooterSubsystem.runShooterMotor(),
            () -> shooterSubsystem.stopShooter(),
            shooterSubsystem
        );
    }
}
