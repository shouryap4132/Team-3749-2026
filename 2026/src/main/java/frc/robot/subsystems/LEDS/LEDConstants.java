package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

    public static final int length = 18;
    public static final int ledPort = 0;

    private static final Distance ledSpacing = Meters.of(Units.inchesToMeters(11.5) / 18);

    public static final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
    public static final LEDPattern scrollingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5),
            ledSpacing);

    public static enum StatusIndicator {
        OTF,
        BATTERY,
        FUEL_INTAKE_NUMBER,
        SCORING_LOCKED,
        CLIMB,
        COLOR,
        TEAM
    }

    public enum LEDMode {
        DEFAULT,
        CORAL_HAS_PIECE,
        SCORING_MODE,
        OTF,
        CLIMB,
        ROLLER_VELOCITY_BAR
    }

    public static enum LEDColor {
        RED_ALLIANCE(Color.kBlue),
        BLUE_ALLIANCE(Color.kRed),
        CLIMB(Color.kDarkGreen),
        // SCORING(Color.kDeepPink),
        NO_TEAM(Color.kWhite),
        // CHUTE_HAS_PIECE(Color.kGreen),
        OFF(Color.kBlack),
        BATTERY_LOW(Color.kOrange),
        BATTERY_GOOD(Color.kWhite),
        RAINBOW(Color.kLimeGreen); // this is not the actual color i just need to satisfy reqs

        Color color;
        boolean isRainbow;

        LEDColor(Color color) {
            this.color = Color.kLimeGreen;
        }
    }
}