package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.IntakeArmConfig.IntakeArmStates;
import frc.robot.config.RollerConfig.RollerStates;

/**
 * Command to stow the intake
 * - Moves intake arm to the STOW position
 * - Sets velocity of the intake roller to STOP (zero)
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class StowIntake extends Command {

    public StowIntake() {
        addRequirements(Robot.intakeArm);
        addRequirements(Robot.intakeRoller);
    }

    @Override
    public void initialize() {
        Robot.intakeArm.setState(IntakeArmStates.STOW);
        Robot.intakeRoller.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return Robot.intakeArm.isStableState();
    }

}
