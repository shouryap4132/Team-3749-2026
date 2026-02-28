package frc.robot.subsystems.LEDS.real;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.LEDS.LEDConstants;
import frc.robot.subsystems.LEDS.LEDIO;

public class LEDReal {

    AddressableLED LEDS;
    AddressableLEDBuffer buffer;

    public LEDReal(int port, AddressableLEDBuffer buffer){
        LEDS = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(port);

        LEDS.setLength(LEDConstants.length);
        LEDS.setData(buffer);
        LEDS.start();

    }

    public void setData(LEDPattern pattern)
    {
        pattern.applyTo(buffer);
        LEDS.setData(buffer);
    }

    
}
