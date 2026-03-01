package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

    private FlywheelIO io;
    private FlywheelIOInputsAutoLogged inputs =
        new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;

        this.setDefaultCommand(running());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Robot/Shooter/Flywheel", inputs);
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Shooter/Flywheel/Idled");
    }

    public Command running() {
        return runOnce(() -> {
                io.setRunning();
            })
            .andThen(idle())
            .withName("Robot/Shooter/Flywheel/Running");
    }
}
