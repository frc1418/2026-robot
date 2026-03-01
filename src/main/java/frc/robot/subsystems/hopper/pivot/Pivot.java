package frc.robot.subsystems.hopper.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;

        this.setDefaultCommand(idled());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Robot/Hopper/IntakePivot", inputs);
    }

    public boolean isDown() {
        return inputs.isDown;
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Hopper/IntakePivot/Idled");
    }

    public Command down() {
        return runOnce(() -> {
                io.setDown();
            })
            .andThen(idle())
            .withName("Robot/Hopper/IntakePivot/Down");
    }
}
