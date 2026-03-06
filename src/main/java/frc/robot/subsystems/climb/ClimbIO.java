package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {

        public boolean motorConnected;
        public double appliedVolts;
        public double currentAmps;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void idle() {}

    public default void declimbALittle() {}

    public default void climbALittle() {}
}
