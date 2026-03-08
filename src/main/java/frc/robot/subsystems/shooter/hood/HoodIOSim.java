package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodIOSim implements HoodIO {

    private double angleDeg = 50;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.outputAngle = Units.degreesToRadians(angleDeg);
    }

    @Override
    public void setIdled() {}

    @Override
    public void setAimed(double angleDeg) {
        this.angleDeg = angleDeg;

        if (this.angleDeg < 50) this.angleDeg = 50;
        if (this.angleDeg > 90) this.angleDeg = 90;
    }

    public double getAngleDeg() {
        return angleDeg;
    }
}
