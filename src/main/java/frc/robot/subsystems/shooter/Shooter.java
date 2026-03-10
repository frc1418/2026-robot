package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

    private Flywheel flywheel;
    private Hood hood;

    public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);

        this.setDefaultCommand(hoodIdled());
    }

    public Command idled() {
        // idle() is required so that these commands take possession of the Shooter subsystem
        return Commands
            .parallel(idle(), flywheel.idled(), hood.idled())
            .withName("Robot/Shooter/Idled");
    }

    public Command hoodIdled() {
        return Commands
            .parallel(idle(), flywheel.running(), hood.idled())
            .withName("Robot/Shooter/HoodIdled");
    }

    public Command hoodAimed(Supplier<Double> angleSupplier) {
        return Commands
            .parallel(idle(), flywheel.running(), hood.aimed(angleSupplier))
            .withName("Robot/Shooter/HoodAimed");
    }

    public Command runShooterFFSysID() {
        return Commands.parallel(
            idle(),
            flywheel.feedforwardCharacterization(),
            hood.idled()
        );
    }

    public double getRPM() {
        return flywheel.getRPM();
    }
}
