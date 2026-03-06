package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class ClimbIOReal implements ClimbIO {

    private SparkMax climbMotor = new SparkMax(16, MotorType.kBrushless);

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        climbMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void idle() {
        Logger.recordOutput("climber/climbing", 0);
        climbMotor.set(0.0);
    }

    @Override
    public void declimbALittle() {
        Logger.recordOutput("climber/climbing", -1);
        climbMotor.set(0.25);
    }

    @Override
    public void climbALittle() {
        Logger.recordOutput("climber/climbing", 1);
        climbMotor.set(-0.5);
    }
}
