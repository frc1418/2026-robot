package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) {
        this.io = io;
        this.setDefaultCommand(doNothing());
    }

    public Command doNothing() {
        return runOnce(() -> {
                io.idle();
            })
            .andThen(idle())
            .withName("Robot/Climber/Idled");
    }

    public Command declimb() {
        return runOnce(() -> {
                io.declimbALittle();
            })
            .andThen(idle())
            .withName("Robot/Climber/Idled");
    }

    public Command climb() {
        return runOnce(() -> {
                io.climbALittle();
            })
            .andThen(idle())
            .withName("Robot/Climber/Idled");
    }
}
