package frc.robot.subsystems.hopper.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public boolean motorConnected;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIdled() {}

    public default void setRunning() {}
}
