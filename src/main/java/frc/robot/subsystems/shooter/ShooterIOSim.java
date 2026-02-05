package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.hopper.HopperIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

    private DCMotor flywheelGearbox = DCMotor.getNeoVortex(2);
    // 40s and higher are reserved for simulation devices
    private SparkFlex flywheelMotor = new SparkFlex(40, MotorType.kBrushless);
    private SparkFlexSim flywheelMotorSim = new SparkFlexSim(
        flywheelMotor,
        flywheelGearbox
    );

    private FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            flywheelGearbox,
            0.00790829399,
            1.0
        ),
        flywheelGearbox
    );

    private HopperIOSim hopperSim;
    private AbstractDriveTrainSimulation driveSim;

    private SparkClosedLoopController flywheelMotorController =
        flywheelMotor.getClosedLoopController();

    private double currentAngle = 60;

    public ShooterIOSim(
        AbstractDriveTrainSimulation driveSim,
        HopperIOSim hopperSim
    ) {
        this.driveSim = driveSim;
        this.hopperSim = hopperSim;

        SparkFlexConfig flywheelMotorConfig = new SparkFlexConfig();
        flywheelMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

        flywheelMotorConfig.closedLoop.p(0.005).i(0.0).d(0.0);
        flywheelMotorConfig.closedLoop.feedForward.kS(0.149).kV(0.00174);

        flywheelMotor.configure(
            flywheelMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterSim.setInput(flywheelMotorSim.getAppliedOutput() * 12.0);
        shooterSim.update(0.02);

        flywheelMotorSim.iterate(
            shooterSim.getAngularVelocityRPM(),
            12.0,
            0.02
        );

        inputs.rpm = shooterSim.getAngularVelocityRPM();
        inputs.appliedVolts = shooterSim.getInputVoltage();

        Logger.recordOutput("ShooterSim/Angle", currentAngle);

        if (inputs.rpm > 300.0) {
            if (hopperSim.transitionFuel()) {
                Pose2d drivePose = driveSim.getSimulatedDriveTrainPose();

                // shoot a fuel
                // exit rpm should be 3850
                RebuiltFuelOnFly shotFuel = new RebuiltFuelOnFly(
                    drivePose.getTranslation(),
                    new Translation2d(0.25, 0.0),
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    drivePose.getRotation(),
                    Inches.of(20),
                    // ≈17.5 m/s @ 4000 RPM
                    MetersPerSecond.of((inputs.rpm / 3000) * 8.0),
                    Degrees.of(currentAngle)
                );

                shotFuel.withTargetPosition(() ->
                    FieldMirroringUtils.toCurrentAllianceTranslation(
                        new Translation3d(4.5974, 4.034536, 1.5748)
                    )
                );
                shotFuel.withTargetTolerance(
                    new Translation3d(0.5969, 0.5969, 0.2)
                );

                shotFuel.withProjectileTrajectoryDisplayCallBack(
                    locations -> {
                        Logger.recordOutput(
                            "ShooterSim/SuccessfulShot",
                            locations.toArray(Pose3d[]::new)
                        );
                    },
                    locations -> {
                        Logger.recordOutput(
                            "ShooterSim/UnsuccessfulShot",
                            locations.toArray(Pose3d[]::new)
                        );
                    }
                );

                Logger.recordOutput(
                    "ShooterSim/ShotDistance",
                    drivePose
                        .getTranslation()
                        .getDistance(new Translation2d(4.5974, 4.03536))
                );

                shotFuel.enableBecomesGamePieceOnFieldAfterTouchGround();

                SimulatedArena.getInstance().addGamePieceProjectile(shotFuel);

                shooterSim.setAngularVelocity(
                    shooterSim.getAngularVelocityRadPerSec() * 0.96375
                );
            }
        }
    }

    @Override
    public void setRPMTarget(double target) {
        flywheelMotorController.setSetpoint(target, ControlType.kVelocity);
    }

    @Override
    public void setVolts(double volts) {
        flywheelMotor.setVoltage(volts);
    }

    @Override
    public double getRPM() {
        return shooterSim.getAngularVelocityRPM();
    }

    @Override
    public void setAngle(double angle) {
        // TODO: maybe a more accurate version of this?
        // this just instantly sets hood angle
        currentAngle = angle;
        if (currentAngle > 80) {
            currentAngle = 80;
        } else if (currentAngle < 40) {
            currentAngle = 40;
        }
    }
}
