package frc.robot.subsystems.hopper.transition;

import org.littletonrobotics.junction.AutoLog;

public interface TransitionIO {
    @AutoLog
    public static class TransitionIOInputs {

        public boolean spindexerConnected;
        public double spindexerVelocityRadPerSecond;
        public double spindexerAppliedVolts;
        public double spindexerCurrentAmps;

        public boolean kickerConnected;
        public double kickerVelocityRadPerSecond;
        public double kickerAppliedVolts;
        public double kickerCurrentAmps;
    }

    public default void updateInputs(TransitionIOInputs inputs) {}

    public default void setIdled() {}

    public default void setRunning() {}
}
