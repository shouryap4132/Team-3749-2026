package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.config.ClimbConfig;
import frc.robot.subsystems.LEDS.LEDConstants.LEDMode;

public class LEDTriggers {

    public static void createLEDTriggers() {

 
        

        



        boolean maxVelocityReached = (Robot.shooterRoller.getVelocity().in(Units.DegreesPerSecond) > 0.1);
        Trigger rollerSpinning = new Trigger( () -> maxVelocityReached );

        rollerSpinning.onTrue(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.ROLLER_VELOCITY_BAR)
            )
        );

        rollerSpinning.onFalse(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.DEFAULT)
            )
        );

 
        // Trigger hasFuel = new Trigger(
        //     () -> Robot.fuel.hasPiece()
        // );

        // hasFuel.onTrue(
        //     Commands.runOnce(() ->
        //         Robot.led.setMode(LEDMode.CORAL_HAS_PIECE)
        //     )
        // );

        // hasFuel.onFalse(
        //     Commands.runOnce(() ->
        //         Robot.led.setMode(LEDMode.DEFAULT)
        //     )
        // );

   
        Trigger otfTrigger = new Trigger(
            () -> Robot.swerve.getIsOTF()
        );

        otfTrigger.onTrue(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.OTF)
            )
        );

        otfTrigger.onFalse(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.DEFAULT)
            )
        );

       
        Trigger climbTrigger = new Trigger(
            () -> Robot.climbLeft.getState().equals(ClimbConfig.ClimbStates.CLIMB)
        );

        climbTrigger.onTrue(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.CLIMB)
            )
        );

        climbTrigger.onFalse(
            Commands.runOnce(() ->
                Robot.led.setMode(LEDMode.DEFAULT)
            )
        );
    }
}