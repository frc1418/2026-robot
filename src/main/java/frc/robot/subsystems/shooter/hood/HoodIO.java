package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {

        public boolean motorConnected;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;
        public double inputAngle;
        public double outputAngle;

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setIdled() {}

    public default void setAimed(double angle) {}

    // public default void prepareHoming() {}

    // public default void setHoming(double volts) {}

    // public default void resetHoming() {}

}
