package frc.robot.subsystems.hopper.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class PivotIOReal implements PivotIO {

    /*
     *  public boolean motorConnected;
        public double angleRad;
        public double velocityRadPerSecond;
        public double appliedVolts;
        public double currentAmps;

        public boolean isDown;
     */

    // 21.375:1 gear ratio

    private enum State {
        IDLED,
        WIGGLING,
        DOWN
    }

    private SparkMax pivotMotor = new SparkMax(14, MotorType.kBrushless);
    private SparkClosedLoopController pivotController =
        pivotMotor.getClosedLoopController();
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private State currentState = State.IDLED;

    private Timer wiggleTimer = new Timer();

    public PivotIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        config.encoder
            .velocityConversionFactor(0.0467836257)
            .positionConversionFactor(0.0467836257);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.5, 0.0, 0.0, ClosedLoopSlot.kSlot0)
            .pid(1.5, 0.0, 0.0, ClosedLoopSlot.kSlot1);
        // TODO: tune this

        pivotMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.angleRad = pivotEncoder.getPosition();
        inputs.velocityRadPerSecond =
            Units.rotationsPerMinuteToRadiansPerSecond(
                pivotEncoder.getVelocity()
            );

        switch (currentState) {
            case IDLED:
                pivotMotor.set(0.0);
                break;
            case WIGGLING:
                pivotController.setSetpoint(
                    (0.915 + 0.305 * Math.cos(wiggleTimer.get() * 3.0)) /
                    Math.PI,
                    ControlType.kPosition
                );
                break;
            case DOWN:
                pivotController.setSetpoint(
                    1.22 / Math.PI,
                    ControlType.kPosition
                );
                break;
        }
    }

    @Override
    public void setIdled() {
        currentState = State.IDLED;
    }

    @Override
    public void setWiggling() {
        currentState = State.WIGGLING;
        wiggleTimer.start();
    }

    @Override
    public void setDown() {
        currentState = State.DOWN;
    }
}
