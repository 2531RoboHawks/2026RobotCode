package frc.robot;
//dissabled - not used currently, only used when we have position commands on intake.java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake;

public class AgitateCommand extends Command {

    private final intake intake;
    private final Timer timer = new Timer();
    private boolean goingDown = true;
    private static final double SWITCH_INTERVAL = 0.3; // seconds between direction changes

    public AgitateCommand(intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        goingDown = true;
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(SWITCH_INTERVAL)) {
            goingDown = !goingDown; // flip direction every 0.3 seconds
        }

        if (goingDown) {
            intake.pivotToDown();
        } else {
            intake.pivotToUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}