package frc.robot.subsystems.hopper.transition;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class TransitionIOReal implements TransitionIO {

    private SparkFlex primary = new SparkFlex(12, MotorType.kBrushless);
    private SparkMax kicker = new SparkMax(13, MotorType.kBrushless);

    public TransitionIOReal() {
        SparkFlexConfig primaryConfig = new SparkFlexConfig();
        SparkMaxConfig kickerConfig = new SparkMaxConfig();

        primaryConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        kickerConfig.apply(primaryConfig);

        primary.configure(
            primaryConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        kicker.configure(
            kickerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(TransitionIOInputs inputs) {
        inputs.kickerConnected = true;
        inputs.spindexerConnected = true;
    }

    @Override
    public void setIdled() {
        primary.set(0.0);
        kicker.set(0.0);
    }

    @Override
    public void setRunning() {
        primary.set(0.15);
        kicker.set(0.5);
    }
}
