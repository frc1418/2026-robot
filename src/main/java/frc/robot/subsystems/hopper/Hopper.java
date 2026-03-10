package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.intake.Intake;
import frc.robot.subsystems.hopper.intake.IntakeIO;
import frc.robot.subsystems.hopper.pivot.Pivot;
import frc.robot.subsystems.hopper.pivot.PivotIO;
import frc.robot.subsystems.hopper.transition.Transition;
import frc.robot.subsystems.hopper.transition.TransitionIO;

public class Hopper extends SubsystemBase {

    private Intake intake;
    private Pivot pivot;
    private Transition transition;

    public Hopper(
        IntakeIO intakeIO,
        PivotIO pivotIO,
        TransitionIO transitionIO
    ) {
        this.intake = new Intake(intakeIO);
        this.pivot = new Pivot(pivotIO);
        this.transition = new Transition(transitionIO);

        this.setDefaultCommand(idled());
    }

    public Command idled() {
        return Commands
            .parallel(idle(), intake.idled(), pivot.down(), transition.idled())
            .withName("Robot/Hopper/Idled");
    }

    // public Command compacted() {
    //     return Commands
    //         .parallel(idle(), intake.idled(), pivot.up(), transition.idled())
    //         .withName("Robot/Hopper/Compacted");
    // }

    public Command intaking() {
        // return Commands
        //     .deadline(
        //         Commands.waitUntil(pivot::isDown),
        //         Commands.parallel(
        //             idle(),
        //             intake.idled(),
        //             //pivot.down(),
        //             transition.idled()
        //         )
        //     )
        //     .andThen(
        //         Commands.parallel(
        //             idle(),
        //             intake.running(),
        //             //pivot.down(),
        //             transition.idled()
        //         )
        //     )
        // return Commands
        //     .parallel(
        //         idle(),
        //         intake.running(),
        //         //pivot.down(),
        //         transition.idled()
        //     )
        //     .withName("Robot/Hopper/Intaking");
        return Commands
            .parallel(
                idle(),
                intake.running(),
                pivot.down(),
                transition.idled()
            )
            .withName("Robot/Hopper/Intaking");
    }

    public Command transitioning() {
        return Commands
            .parallel(
                idle(),
                intake.slowRunning(),
                pivot.wiggling(),
                transition.running()
            )
            .withName("Robot/Hopper/Transitioning");
    }
}
