package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LedUtils;
import java.util.function.DoubleSupplier;

public class Led extends SubsystemBase {

    private static double progress = 0.0;

    public static enum Pattern {
        DRIVING_TO_SPOT(
            LEDPattern
                .rainbow(255, 255)
                .scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(0.75),
                    Centimeters.of(100 / 60)
                )
        ),
        AIMING(
            LedUtils.inOutProgressSwitcher(
                () -> progress,
                Color.kGreen,
                Color.kBlack
            )
        ),
        SHOOTING(
            LEDPattern
                .gradient(
                    GradientType.kContinuous,
                    Color.kGreen,
                    Color.kGreenYellow
                )
                .scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(0.75),
                    Centimeters.of(100 / 60)
                )
        ),
        // CURRENT_SHIFT(LedUtils.allianceShiftPattern()),
        DISABLED(
            LEDPattern
                .gradient(
                    GradientType.kContinuous,
                    Color.kBlue,
                    Color.kRed,
                    Color.kBlue,
                    Color.kRed
                )
                .scrollAtAbsoluteSpeed(
                    MetersPerSecond.of(0.25),
                    Centimeters.of(100 / 60)
                )
        );

        LEDPattern pattern;

        private Pattern(LEDPattern pattern) {
            this.pattern = pattern;
        }
    }

    private LedIO io;
    private Pattern pattern;

    public Led(LedIO io) {
        this.io = io;

        this.pattern = Pattern.DISABLED;

        this.setDefaultCommand(shiftOrDisabled());
    }

    @Override
    public void periodic() {
        io.updatePattern(pattern.pattern);
    }

    public Command shiftOrDisabled() {
        return run(() -> {
            if (DriverStation.isEnabled()) {
                pattern = Pattern.DISABLED;
            } else {
                pattern = Pattern.DISABLED;
            }
        });
    }

    public Command drivingToSpot() {
        return runOnce(() -> pattern = Pattern.DRIVING_TO_SPOT).andThen(idle());
    }

    public Command aiming(DoubleSupplier progress) {
        return runOnce(() -> pattern = Pattern.AIMING)
            .andThen(run(() -> Led.progress = progress.getAsDouble()));
    }

    public Command shooting() {
        return runOnce(() -> pattern = Pattern.SHOOTING).andThen(idle());
    }
}
