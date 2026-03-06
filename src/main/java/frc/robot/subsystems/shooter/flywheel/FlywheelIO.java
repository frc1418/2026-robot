package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {

        public boolean motorsConnected;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setIdled() {}

    public default void setRunning() {}

    public default void setVoltageForSysID(double voltage) {}
}
