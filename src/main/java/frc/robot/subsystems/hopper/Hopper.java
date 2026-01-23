package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public Command runIntake() {
        return run(() -> {
            this.io.setIntakeRunning(true);
            this.io.setTransitionRunning(false);
        });
    }

    public Command runTransition() {
        return run(() -> {
            this.io.setIntakeRunning(false);
            this.io.setTransitionRunning(true);
        });
    }

    public Command idle() {
        return run(() -> {
            this.io.setIntakeRunning(false);
            this.io.setTransitionRunning(false);
        });
    }
}
