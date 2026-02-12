package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.RollerConfig.RollerStates;

/**
 * Command to stop the hopper motors
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class StopHopper extends Command {

    public StopHopper() {
        addRequirements(Robot.hopperRollers);
    }

    @Override
    public void initialize() {
        Robot.hopperRollers.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
