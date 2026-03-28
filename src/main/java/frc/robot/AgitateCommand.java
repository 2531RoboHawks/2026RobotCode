package frc.robot;
//dissabled - not used currently, only used when we have position commands on intake.java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class AgitateCommand extends Command {

    private final Intake intake;
    private final Timer timer = new Timer();
    private boolean goingDown = true;
    public AgitateCommand(Intake intake) {
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
        if (timer.advanceIfElapsed(Constants.Agitate.kSwitchInterval)) {
            goingDown = !goingDown;
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