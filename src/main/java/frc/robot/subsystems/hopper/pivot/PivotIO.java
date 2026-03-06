package frc.robot.subsystems.hopper.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {

        public boolean motorConnected;
        public double angleRad;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;

        public boolean isDown;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setIdled() {}

    public default void setWiggling() {}

    public default void setDown() {}
}
