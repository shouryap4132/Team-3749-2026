package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.IntakeArmConfig.IntakeArmStates;
import frc.robot.config.RollerConfig.RollerStates;

/**
 * Command to deploy the intake
 * - Moves intake arm to the DEPLOYED position
 * - Sets velocity of the intake roller to INTAKE (defined in RollerConfig)
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class DeployIntake extends Command {

    public DeployIntake() {
        addRequirements(Robot.intakeArm);
        addRequirements(Robot.intakeRoller);
    }

    @Override
    public void initialize() {
        Robot.intakeArm.setState(IntakeArmStates.DEPLOYED);
    }

    @Override
    public void execute(){
        if (Robot.intakeArm.isStableState()){
            Robot.intakeRoller.setState(RollerStates.INTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intakeArm.isStableState() && Robot.intakeRoller.getState() == RollerStates.INTAKE;
    }

}
