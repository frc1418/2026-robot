package frc.robot.subsystems.hopper.pivot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotIOReal implements PivotIO {

    /*
     *  public boolean motorConnected;
        public double angleRad;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;

        public boolean isDown;
     */

    private SparkMax pivotMotor = new SparkMax(14, MotorType.kBrushless);
    private SparkClosedLoopController pivotController =
        pivotMotor.getClosedLoopController();

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        config.encoder
            .velocityConversionFactor(16 / 38 / 9)
            .positionConversionFactor(16 / 38 / 9);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.25, 0.0, 0.0);
        // only use S, V, and C (A should be 0.0, R should be 1.0)
        // TODO: tune this
        config.closedLoop.feedForward.svacr(0.0, 0.0, 0.0, 0.0, 1.0);
    }

    @Override
    public void setIdled() {}

    @Override
    public void setDown() {
        pivotController.setSetpoint(0.0, ControlType.kPosition);
    }
}
