package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        public double rpm = 0.0;
        public double appliedVolts = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setRPMTarget(double rpm) {}

    public default void setVolts(double volts) {}

    public default double getRPM() {
        return 0.0;
    }

    public default void setAngle(double angle) {}
}
