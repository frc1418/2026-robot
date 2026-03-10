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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodIOReal implements HoodIO {

    private final InterpolatingDoubleTreeMap LUT = new InterpolatingDoubleTreeMap();
    {
        LUT.put(2.473, 0.375);
        LUT.put(2.73, 0.304);
        LUT.put(3.016, 0.267);
        LUT.put(3.517, 0.221);
        LUT.put(3.717, 0.194);
        LUT.put(4.03, 0.158);
        LUT.put(4.515, 0.121);
        LUT.put(4.736, 0.086);
        LUT.put(4.995, 0.066);
    }

    // private final double GEAR_RATIO = 9 * (184.0 / 14.0); // 9:1 from motor to output shaft, then 184 teeth to 14 teeth
    private final double GEAR_RATIO = 9.0 * (184.0 / 14.0);
    private final double rotsToRads = (2 * Math.PI / GEAR_RATIO);

    private SparkMax hoodMotor = new SparkMax(11, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private SparkClosedLoopController hoodController =
        hoodMotor.getClosedLoopController();

    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    public HoodIOReal() {
        hoodConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(true);

        hoodConfig.encoder
            .positionConversionFactor(rotsToRads)
            .velocityConversionFactor(rotsToRads / 60.0);

        hoodConfig.closedLoop.pid(5.0, 0, 0);

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
        hoodController.setSetpoint(angleRad, ControlType.kPosition);
    }

    @Override
    public void aimHood(double distanceToHub) {
        double targetAngle = LUT.get(distanceToHub);
        setAimed(targetAngle);
    }
}
