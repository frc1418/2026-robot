package frc.robot.subsystems.hopper.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hopper.pivot.PivotIOSim;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {

    private IntakeSimulation simulation;
    private PivotIOSim pivotSim;

    private boolean shouldRun = false;
    private boolean canRun = false;

    public IntakeIOSim(
        AbstractDriveTrainSimulation driveTrainSimulation,
        PivotIOSim pivot
    ) {
        double lengthExtended = Units.inchesToMeters(7.3);
        double width = Units.inchesToMeters(17.5);
        double xOffset = Units.inchesToMeters(-2.75);

        Rectangle intakeShape = new Rectangle(lengthExtended, width);
        intakeShape.translate(
            -driveTrainSimulation.config.bumperLengthX.in(Meters) *
            0.5 -
            lengthExtended *
            0.5 +
            0.01,
            xOffset
        );
        this.simulation =
            new IntakeSimulation("Fuel", driveTrainSimulation, intakeShape, 3);

        this.pivotSim = pivot;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.velocityRadPerSecond = simulation.isRunning() ? 1000 : 0;

        boolean canRunCurrent = pivotSim.getIsDown() && shouldRun;

        if (canRunCurrent && !canRun) {
            simulation.startIntake();
        }

        if (!canRunCurrent && canRun) {
            simulation.stopIntake();
        }

        canRun = canRunCurrent;
    }

    @Override
    public void setIdled() {
        shouldRun = false;
    }

    @Override
    public void setRunning() {
        shouldRun = true;
    }

    public int moveToTransition() {
        int amount = simulation.getGamePiecesAmount();
        simulation.setGamePiecesCount(0);
        return amount;
    }
}
