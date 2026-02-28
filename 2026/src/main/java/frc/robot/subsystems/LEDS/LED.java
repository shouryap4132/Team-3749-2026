package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDS.LEDConstants.LEDColor;
import frc.robot.subsystems.LEDS.LEDConstants.LEDMode;
import frc.robot.subsystems.LEDS.LEDConstants.StatusIndicator;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDS.LEDTriggers;

import frc.robot.subsystems.LEDS.real.LEDReal;

public class LED extends SubsystemBase {
    private AddressableLEDBuffer ledBuffer;

    private LEDColor desiredColor = getTeamColorLED();
    private LEDColor currentColor = null;

    private boolean doRainbow = false;

    private LEDMode currentMode = LEDMode.DEFAULT;


    private StatusIndicator statusIndicator = StatusIndicator.FUEL_INTAKE_NUMBER;

    private double brightness = 1;

    private LEDReal ledBase;

    public LED() {
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        if (Robot.isReal()) {
            ledBase = new LEDReal(LEDConstants.ledPort, ledBuffer);
        }
        ledBase.setData(LEDPattern.kOff);
        LEDTriggers.createLEDTriggers();
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public LED(double brightness) {
        this();
        this.brightness = brightness;
    }

    /**
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDColor getCurrentColor() {
        return currentColor;
    }

    /**
     * Returns a LED pattern that matches the current alliance of the robot
     * 
     * @return
     */
    public LEDColor getTeamColorLED() {
        Optional<Alliance> team = DriverStation.getAlliance(); // i hate doing it this way but it throws an error
                                                               // without it
        if (!team.isPresent()) {
            return LEDColor.NO_TEAM;
        }

        return team.get() == Alliance.Blue ? LEDColor.BLUE_ALLIANCE : LEDColor.RED_ALLIANCE;
    }



    public void setMode(LEDMode mode) {
        currentMode = mode;
    }

    private static final int ROLLER_START = 0;
    private static final int ROLLER_LENGTH = 20;

    private void updateRollerVelocityBar() {
        double velocity = Robot.shooterRoller.getVelocity();
        double maxVelocity = Robot.shooterRoller.getMaxVelocity();

        double percent = MathUtil.clamp(velocity / maxVelocity, 0.0, 1.0);
        int litLEDs = (int) Math.round(percent * ROLLER_LENGTH);

        for (int i = 0; i < ROLLER_LENGTH; i++) {
            int idx = ROLLER_START + i;
            if (i < litLEDs) {
                ledBuffer.setLED(idx, LEDConstants.LEDColor.RAINBOW.color);
            } else {
                ledBuffer.setRGB(idx, 0, 0, 0);
            }
        }
}

    private void setStripColor() {
        switch (statusIndicator) {
            // case CORAL_PIECE:
            //     desiredColor = LEDColor.CORAL_ARM_HAS_PIECE;
            //     doRainbow = Robot.coralRoller.hasPiece();
            //     break;
            // default:
            //     desiredColor = getTeamColorLED();
            //     doRainbow = false;
            //     break;
        }
    }

    public void setLEDColor(LEDColor color) {
        this.desiredColor = color;
    }

    public void setLEDStatusIndicator(StatusIndicator indicator) {
        statusIndicator = indicator;
    }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    public void updateLEDs() {
        if (desiredColor == currentColor) {
            return;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredColor.color).atBrightness(Percent.of(brightness * 100));

        if(doRainbow) {
            setPattern = LEDConstants.scrollingRainbow;
        }

        ledBase.setData(setPattern);
        currentColor = desiredColor;
    }

    public void logData() {
        Logger.recordOutput("LED/currentPattern", currentColor.toString());
        Logger.recordOutput("LED/desiredPattern", desiredColor.toString());
        Logger.recordOutput("LED/brightness", brightness);
    }

    // @Override
    // public void periodic() {
    //     setStripColor();
    //     updateLEDs();
    //     logData();
    // }

    @Override
    public void periodic() {
        switch (currentMode) {
            case ROLLER_VELOCITY_BAR:
                updateRollerVelocityBar();
                break;

            case CLIMB:
                LEDPattern.solid(LEDColor.CLIMB.color);
                break;

            default:
                LEDPattern.solid(getTeamColorLED().color);
                break;
        }

        updateLEDs();
        logData();
    }

}