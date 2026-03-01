package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class HopperIOSim implements HopperIO {

    private IntakeSimulation simulation;

    private Timer transitionTimer;
    private boolean isTransitionRunning;

    public HopperIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.simulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrainSimulation,
                Inches.of(25),
                Inches.of(10),
                IntakeSimulation.IntakeSide.BACK,
                40
            );

        this.transitionTimer = new Timer();
        this.transitionTimer.start();
        this.isTransitionRunning = false;
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.isIntakeRunning = this.simulation.isRunning();
        inputs.isTransistionRunning = this.isTransitionRunning;

        Logger.recordOutput(
            "IntakeSim/currentFuel",
            this.simulation.getGamePiecesAmount()
        );
    }

    @Override
    public void setIntakeRunning(boolean running) {
        if (running) {
            this.simulation.startIntake();
        } else {
            this.simulation.stopIntake();
        }
    }

    @Override
    public void setTransitionRunning(boolean running) {
        this.isTransitionRunning = running;
    }

    public boolean transitionFuel() {
        if (transitionTimer.hasElapsed(0.2) && this.isTransitionRunning) {
            transitionTimer.restart();
            return this.simulation.obtainGamePieceFromIntake();
        } else {
            return false;
        }
    }
}
