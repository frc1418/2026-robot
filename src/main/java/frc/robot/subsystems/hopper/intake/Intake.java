package frc.robot.subsystems.hopper.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;

        this.setDefaultCommand(idled());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Robot/Hopper/Intake", inputs);
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Hopper/Intake/Idled");
    }

    public Command slowRunning() {
        return runOnce(() -> {
                io.setRunningSlow();
            })
            .andThen(idle())
            .withName("Robot/Hopper/Intake/Running");
    }

    public Command running() {
        return runOnce(() -> {
                io.setRunning();
            })
            .andThen(idle())
            .withName("Robot/Hopper/Intake/Running");
    }
}
