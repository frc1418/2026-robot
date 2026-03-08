package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HoodIOReal implements HoodIO {

    // private final double GEAR_RATIO = 9 * (184.0 / 14.0); // 9:1 from motor to output shaft, then 184 teeth to 14 teeth
    private final double GEAR_RATIO = 9.0 * (184.0 / 14.0);
    private final double rotsToRads = (2 * Math.PI / GEAR_RATIO) * 1.85;

    private SparkMax hoodMotor = new SparkMax(11, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private SparkClosedLoopController hoodController =
        hoodMotor.getClosedLoopController();

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    SparkMaxConfig homingConfig = new SparkMaxConfig();

    public HoodIOReal() {
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        hoodConfig.encoder
            .positionConversionFactor(rotsToRads)
            .velocityConversionFactor(rotsToRads / 60.0);

        hoodConfig.closedLoop.pid(0.8, 0, 0).outputRange(-1.0, 1.0);

        homingConfig.apply(hoodConfig);
        homingConfig.smartCurrentLimit(5);

        hoodMotor.configure(
            hoodConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        hoodEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = hoodMotor.getBusVoltage() > 0.0;
        inputs.velocityRadPerSecond = hoodEncoder.getVelocity();
        inputs.appliedVolts =
            hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.currentAmps = hoodMotor.getOutputCurrent();
        inputs.inputAngle = hoodController.getSetpoint();
        inputs.outputAngle = hoodEncoder.getPosition();
    }

    @Override
    public void setIdled() {
        double currentAngle = hoodEncoder.getPosition();
        hoodController.setSetpoint(currentAngle, ControlType.kPosition);
    }

    @Override
    public void setAimed(double angleRad) {
        hoodController.setSetpoint(-angleRad, ControlType.kPosition);
    }

    // @Override
    // public void prepareHoming() {
    //     hoodMotor.configure(homingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // }

    // @Override
    // public void setHoming(double volts) {
    //     hoodMotor.set(volts);
    // }

    // @Override
    // public void resetHoming() {
    //     hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     hoodEncoder.setPosition(0.0);
    // }

}
