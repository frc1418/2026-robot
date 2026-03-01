package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;

        this.setDefaultCommand(idled());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Robot/Shooter/Hood", inputs);
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Shooter/Hood/Idled");
    }

    public Command aimed(Supplier<Double> angleSupplier) {
        return run(() -> {
                io.setAimed(angleSupplier.get());
            })
            .withName("Robot/Shooter/Hood/Aimed");
    }
}
