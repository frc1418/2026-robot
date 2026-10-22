package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class LedIOReal implements LedIO {

    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(120);

    public LedIOReal() {
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void updatePattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
        leds.setData(buffer);
    }
}
