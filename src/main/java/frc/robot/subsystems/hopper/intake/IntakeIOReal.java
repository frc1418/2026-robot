package frc.robot.subsystems.hopper.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {

    private SparkMax motor = new SparkMax(15, MotorType.kBrushless);
    private SparkMax kicker = new SparkMax(17, MotorType.kBrushless);
    private RelativeEncoder speedEncoder = motor.getEncoder();

    public IntakeIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        kicker.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        ifOk(
            motor,
            speedEncoder::getVelocity,
            (double value) -> {
                inputs.velocityRadPerSecond = value;
            }
        );
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorConnected = !sparkStickyFault;
    }

    @Override
    public void setIdled() {
        motor.set(0.0);
        kicker.set(0.0);
    }

    @Override
    public void setRunningSlow() {
        motor.set(0.5);
        kicker.set(-0.5);
    }

    @Override
    public void setRunning() {
        motor.set(1.0);
        kicker.set(-0.5);
    }
}
