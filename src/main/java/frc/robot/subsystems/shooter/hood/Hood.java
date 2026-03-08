package frc.robot.subsystems.shooter.hood;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    // private final Timer stallTimer = new Timer();

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
    
    // public Command home() {
    //     return run(() -> {
    //         io.setHoming(0.1); // change to positive if needed and maybe reduce/increase voltage (test ts)
    //     }).beforeStarting(() -> {
    //         io.prepareHoming();
    //         stallTimer.reset();
    //         stallTimer.start();
    //     }).until(() -> {
    //         return Math.abs(inputs.velocityRadPerSecond) < 0.5 && stallTimer.hasElapsed(0.3);
    //     }).finallyDo((interrupted) -> {
    //         io.setHoming(0.0);
    //         io.resetHoming();
    //         stallTimer.stop();
    //     }).withName("Robot/Shooter/Hood/Homing");
    // }

}
