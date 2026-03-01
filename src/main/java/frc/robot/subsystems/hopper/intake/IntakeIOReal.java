package frc.robot.subsystems.hopper.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOReal implements IntakeIO {

    private SparkFlex motor = new SparkFlex(15, MotorType.kBrushless);
    private RelativeEncoder speedEncoder = motor.getEncoder();

    public IntakeIOReal() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);

        motor.configure(
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
    }

    @Override
    public void setRunning() {
        motor.set(0.5);
    }
}
