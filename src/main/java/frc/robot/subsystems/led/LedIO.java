package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LedIO {
    public default void updatePattern(LEDPattern pattern) {}
}
