package frc.robot.subsystems.hopper.transition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Transition extends SubsystemBase {

    private TransitionIO io;
    private TransitionIOInputsAutoLogged inputs =
        new TransitionIOInputsAutoLogged();

    public Transition(TransitionIO io) {
        this.io = io;

        this.setDefaultCommand(idled());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Robot/Hopper/Transition", inputs);
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Hopper/Transition/Idled");
    }

    public Command running() {
        return run(() -> {
                // if (
                //     Math.abs(shooter.getRPM() - ShooterConstants.targetRPM) < 30
                // ) {
                //     io.setRunning();
                // } else {
                //     io.setIdled();
                // }
                io.setRunning();
            })
            .withName("Robot/Hopper/Transition/Running");
    }
}
