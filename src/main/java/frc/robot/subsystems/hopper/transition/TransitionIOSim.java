package frc.robot.subsystems.hopper.transition;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.hopper.intake.IntakeIOSim;
import org.littletonrobotics.junction.Logger;

public class TransitionIOSim implements TransitionIO {

    private boolean unlimited;

    private Timer transitionTimer = new Timer();
    private int heldFuelCount = 0;
    private boolean isTransitionRunning = false;

    private IntakeIOSim intakeSim;

    public TransitionIOSim(IntakeIOSim intakeSim, boolean unlimited) {
        this.unlimited = unlimited;
        this.intakeSim = intakeSim;
        transitionTimer.start();
    }

    @Override
    public void updateInputs(TransitionIOInputs inputs) {
        inputs.spindexerConnected = true;
        inputs.kickerConnected = true;

        int intakedFuelCount = intakeSim.moveToTransition();
        heldFuelCount += intakedFuelCount;
        if (heldFuelCount > 30) {
            //int extraFuelCount = heldFuelCount - 40;
            heldFuelCount = 30;
        }

        Logger.recordOutput("Simulation/Transition/HeldFuel", heldFuelCount);
        Logger.recordOutput(
            "Simulation/Transition/isRunning",
            isTransitionRunning
        );
    }

    @Override
    public void setIdled() {
        isTransitionRunning = false;
    }

    @Override
    public void setRunning() {
        isTransitionRunning = true;
    }

    public boolean transitionFuel() {
        if (
            transitionTimer.hasElapsed(0.15) &&
            this.isTransitionRunning &&
            (heldFuelCount > 0 || unlimited)
        ) {
            transitionTimer.restart();
            heldFuelCount--;
            return true;
        } else {
            return false;
        }
    }

    public boolean isUnlimited() {
        return unlimited;
    }
}
