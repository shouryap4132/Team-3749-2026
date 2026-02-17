package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.ClimbConfig.ClimbStates;


/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class Climb extends Command {

    public Climb() {
        addRequirements(Robot.climbLeft);
    }

    @Override
    public void initialize() {
        Robot.climbLeft.setState(ClimbStates.CLIMB);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}