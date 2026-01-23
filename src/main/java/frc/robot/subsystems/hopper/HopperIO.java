package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {

        public boolean isIntakeRunning = false;
        public boolean isTransistionRunning = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(HopperIOInputs inputs) {}

    public default void setIntakeRunning(boolean running) {}

    public default void setTransitionRunning(boolean running) {}
}
