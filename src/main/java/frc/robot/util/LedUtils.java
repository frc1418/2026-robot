package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

public class LedUtils {

    public static LEDPattern inOutProgressMaskLayer(
        DoubleSupplier progressSupplier
    ) {
        return (reader, writer) -> {
            double progress =
                MathUtil.clamp(progressSupplier.getAsDouble(), 0, 1) * 0.5;

            int bufLen = reader.getLength();
            int first = (int) (bufLen * progress);
            int last = (int) (bufLen * (1.0 - progress));

            for (int led = 0; led < first; led++) {
                writer.setLED(led, Color.kWhite);
            }

            for (int led = first; led < last; led++) {
                writer.setLED(led, Color.kBlack);
            }

            for (int led = last; led < bufLen; led++) {
                writer.setLED(led, Color.kWhite);
            }
        };
    }

    public static LEDPattern inOutProgressSwitcher(
        DoubleSupplier progressSupplier,
        Color color0,
        Color color1
    ) {
        return (reader, writer) -> {
            double progress =
                MathUtil.clamp(progressSupplier.getAsDouble(), 0, 1) * 0.5;

            int bufLen = reader.getLength();
            int first = (int) (bufLen * progress);
            int last = (int) (bufLen * (1.0 - progress));

            for (int led = 0; led < first; led++) {
                writer.setLED(led, color0);
            }

            for (int led = first; led < last; led++) {
                writer.setLED(led, color1);
            }

            for (int led = last; led < bufLen; led++) {
                writer.setLED(led, color0);
            }
        };
    }

    public static LEDPattern allianceShiftPattern() {
        return (reader, writer) -> {
            // current shift from matchutils
            MatchUtils.Shift currentShift = MatchUtils.getCurrentShift();

            // progress countdown (full -> empty as shift ends)
            double progress = 1.0 - currentShift.getProgress();

            // current color based on which alliance is active
            // blue = blue, red = red, purple = both, orange = neither
            Color currentColor;
            switch (currentShift.alliance()) {
                case BLUE:
                    currentColor = Color.kBlue;
                    break;
                case RED:
                    currentColor = Color.kRed;
                    break;
                case BOTH:
                    currentColor = Color.kPurple;
                    break;
                case NEITHER:
                    currentColor = Color.kOrangeRed;
                    break;
                default:
                    currentColor = Color.kBlack;
                    break;
            }

            // clamp progress to [0, 1] and halve (since it fills in out to in)
            double stripProgress = MathUtil.clamp(progress, 0, 1) * 0.5;

            // get locations on the strip where the leds will switch color
            int bufLen = reader.getLength();
            int first = (int) (bufLen * stripProgress);
            int last = (int) (bufLen * (1.0 - stripProgress));

            // current alliance color, black, current alliance color
            for (int led = 0; led < first; led++) {
                writer.setLED(led, currentColor);
            }

            for (int led = first; led < last; led++) {
                writer.setLED(led, Color.kBlack);
            }

            for (int led = last; led < bufLen; led++) {
                writer.setLED(led, currentColor);
            }
        };
    }
}
