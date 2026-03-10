package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.ShooterConstants.targetRPM;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class FlywheelIOReal implements FlywheelIO {

    private SparkFlex leftFlywheel = new SparkFlex(9, MotorType.kBrushless);
    private SparkFlex rightFlywheel = new SparkFlex(10, MotorType.kBrushless);

    private RelativeEncoder flywheelEncoder = leftFlywheel.getEncoder();
    private SparkClosedLoopController flywheelController =
        leftFlywheel.getClosedLoopController();

    private boolean reversed = false;
    private boolean isRunning = false;
    private boolean isUsingVoltage = false;

    public FlywheelIOReal() {
        SparkFlexConfig leftConfig = new SparkFlexConfig();
        SparkFlexConfig rightConfig = new SparkFlexConfig();

        leftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
        leftConfig.closedLoop.pid(0.0004, 0, 0);
        leftConfig.closedLoop.feedForward.sv(0.22560, 0.00183);

        rightConfig.apply(leftConfig).follow(leftFlywheel, true);

        leftFlywheel.configure(
            leftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        rightFlywheel.configure(
            rightConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (!isUsingVoltage) {
            if (isRunning) {
                if (reversed) {
                    leftFlywheel.set(-0.5);
                } else {
                    flywheelController.setSetpoint(
                        targetRPM,
                        ControlType.kVelocity
                    );
                    // leftFlywheel.set(0.35);
                }
            } else {
                leftFlywheel.set(0.0);
            }
        }
        ifOk(
            leftFlywheel,
            flywheelEncoder::getVelocity,
            value ->
                inputs.velocityRadPerSecond =
                    Units.rotationsPerMinuteToRadiansPerSecond(value)
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
        isUsingVoltage = false;
        isRunning = false;
        reversed = false;
    }

    @Override
    public void setRunning() {
        isUsingVoltage = false;
        isRunning = true;
        reversed = false;
    }

    @Override
    public void setUnsticking() {
        isUsingVoltage = false;
        isRunning = true;
        reversed = true;
    }

    @Override
    public void setVoltageForSysID(double voltage) {
        isUsingVoltage = true;
        leftFlywheel.setVoltage(voltage);
    }
}
