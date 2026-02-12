package frc.robot.commands.HoodedShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.HoodedShooterConfig.HoodedShooterStates;

public class Pass extends Command{

    @Override
    public void initialize() {
        addRequirements(Robot.hoodedShooter);
        Robot.hoodedShooter.setState(HoodedShooterStates.PASS);
        Robot.swerve.setIsPassing(true);
        Robot.swerve.setIsAutoAim(false);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished() {
        return Robot.hoodedShooter.getState() != HoodedShooterStates.PASS;
    }
}
