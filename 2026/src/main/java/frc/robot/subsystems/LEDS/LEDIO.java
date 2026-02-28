package frc.robot.subsystems.LEDS;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LEDIO {
    public default void setData(LEDPattern pattern){}
}