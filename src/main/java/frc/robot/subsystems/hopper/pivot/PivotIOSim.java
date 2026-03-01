package frc.robot.subsystems.hopper.pivot;

public class PivotIOSim implements PivotIO {

    private boolean isDown = false;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.isDown = isDown;
        inputs.motorConnected = true;
    }

    @Override
    public void setIdled() {}

    @Override
    public void setDown() {
        isDown = true;
    }

    public boolean getIsDown() {
        return isDown;
    }
}
