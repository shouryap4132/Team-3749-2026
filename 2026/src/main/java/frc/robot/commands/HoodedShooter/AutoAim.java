package frc.robot.commands.HoodedShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.HoodedShooterConfig.HoodedShooterStates;

/**
 * Command that enables the hooded shooter AUTOAIM behavior and notifies the swerve subsystem
 * that the robot is currently auto-aiming.
 *
 * <p>Lifecycle:
 * - initialize(): set the HoodedShooter state to AUTOAIM and enable the swerve auto-aim flag.
 * - execute(): intentionally empty (HoodedShooter handles per-loop AUTOAIM behavior in periodic()).
 * - end(): put hood back into STOW when the command ends or is interrupted.
 * - isFinished(): command finishes when the swerve subsystem clears the auto-aim flag.
 *
 * <p>Notes:
 * - This command is a lightweight coordinator; the actual angle computation and hood control
 *   are performed inside the HoodedShooter subsystem when its state is AUTOAIM.
 */
public class AutoAim extends Command{

    /**
     * Called once when the command is initially scheduled.
     *
     * Sets the HoodedShooter state to AUTOAIM and flips the swerve auto-aim flag so the
     * drivetrain can perform any required auto-aim behavior (e.g., locking rotation).
     */
    @Override
    public void initialize() {
        addRequirements(Robot.hoodedShooter);
        Robot.hoodedShooter.setState(HoodedShooterStates.AUTOAIM);
    }

    /**
     * Called repeatedly while the command is scheduled.
     *
     * This command intentionally does not perform per-loop work; the HoodedShooter subsystem's
     * periodic() handles computing shooter angles and driving the hood when in AUTOAIM state.
     */
    @Override
    public void execute() {}

    /**
     * Called once when the command ends or is interrupted.
     *
     * @param interrupted true if the command was canceled/interrupted, false if it finished normally.
     *
     * On end we return the hood to the STOW state as a safe default.
     */
    @Override
    public void end(boolean interrupted) {}

    /**
     * Returns whether the command has finished.
     *
     * <p>This command remains scheduled while the swerve subsystem's auto-aim flag is set.
     * When swerve clears that flag (for example by operator action), this method returns true
     * and the command will finish.
     *
     * @return true when swerve has cleared auto-aim, false otherwise
     */
    @Override
    public boolean isFinished() {
        return Robot.hoodedShooter.getState() != HoodedShooterStates.AUTOAIM;
    }
}
