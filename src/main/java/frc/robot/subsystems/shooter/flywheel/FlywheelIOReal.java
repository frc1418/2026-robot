package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class FlywheelIOReal implements FlywheelIO {

    private SparkFlex leftFlywheel = new SparkFlex(9, MotorType.kBrushless);
    private SparkFlex rightFlywheel = new SparkFlex(10, MotorType.kBrushless);

    private RelativeEncoder flywheelEncoder = leftFlywheel.getEncoder();

    public boolean isRunning = false;

    public FlywheelIOReal() {

        SparkFlexConfig leftConfig = new SparkFlexConfig();
        SparkFlexConfig rightConfig = new SparkFlexConfig();

        leftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
        rightConfig.apply(leftConfig).inverted(true).follow(leftFlywheel);

        leftFlywheel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFlywheel.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {

        if(!isRunning) {
            leftFlywheel.set(0.0);
        } else if(flywheelEncoder.getVelocity() < 3000) {
            leftFlywheel.set(1.0);
        } else {
            // FF = sv(0.0149, 0.00174) [values maybe need to be updated; calculated from sim rn]
            // calculated % from ff 
            // (0.0149 + 0.00174*3000)/12 = 0.436241667
            // there's more static friction than in sim, so a tiny bit more to compensate
            leftFlywheel.set(0.45);
        }

        ifOk(
            leftFlywheel,
            flywheelEncoder::getVelocity,
            value -> inputs.velocityRadPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(value)
        );
        ifOk(
            leftFlywheel,
            new DoubleSupplier[] {
                leftFlywheel::getAppliedOutput,
                leftFlywheel::getBusVoltage
            },
            values -> inputs.appliedVolts = values[0] * values[1]
        );
        ifOk(
            leftFlywheel,
            leftFlywheel::getOutputCurrent,
            value -> inputs.currentAmps = value
        );
        inputs.motorsConnected = !sparkStickyFault;

    }

    @Override
    public void setIdled() {
        isRunning = false;
    }

    @Override
    public void setRunning() {
        isRunning = true;
    }

    
}
