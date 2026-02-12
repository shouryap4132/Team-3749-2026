package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.RollerConfig.RollerStates;

/**
 * Command to run the hopper motors
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class RunHopper extends Command {

    public RunHopper() {
        addRequirements(Robot.hopperRollers);
    }

    @Override
    public void initialize() {
        Robot.hopperRollers.setState(RollerStates.RUN_HOPPER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
