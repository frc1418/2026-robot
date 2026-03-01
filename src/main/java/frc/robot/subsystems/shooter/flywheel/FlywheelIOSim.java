package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.subsystems.hopper.transition.TransitionIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class FlywheelIOSim implements FlywheelIO {

    private AbstractDriveTrainSimulation driveSim;
    private HoodIOSim hoodSim;
    private TransitionIOSim transitionSim;

    // 40s and higher are reserved for simulation devices
    private SparkFlex flywheelMotor = new SparkFlex(40, MotorType.kBrushless);
    private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
    private SparkClosedLoopController flywheelController =
        flywheelMotor.getClosedLoopController();

    private boolean isRunning = false;

    private DCMotor flywheelGearbox = DCMotor.getNeoVortex(2);
    private SparkFlexSim flywheelMotorSim = new SparkFlexSim(
        flywheelMotor,
        flywheelGearbox
    );
    private FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(flywheelGearbox, 0.008, 1.0),
        flywheelGearbox
    );

    public FlywheelIOSim(
        AbstractDriveTrainSimulation driveSim,
        HoodIOSim hoodSim,
        TransitionIOSim transitionSim
    ) {
        this.driveSim = driveSim;
        this.hoodSim = hoodSim;
        this.transitionSim = transitionSim;

        SparkFlexConfig flywheelMotorConfig = new SparkFlexConfig();
        flywheelMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);

        flywheelMotorConfig.closedLoop.pid(0.0015, 0.0, 0.0);
        // flywheelMotorConfig.closedLoop.feedForward.sv(0.0149, 0.00174);
        flywheelMotorConfig.closedLoop.feedForward.sva(0.0, 0.22, 2.69);

        flywheelMotorConfig.closedLoop.maxMotion
            .cruiseVelocity(3000)
            .maxAcceleration(1500);

        flywheelMotor.configure(
            flywheelMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        shooterSim.setInput(flywheelMotorSim.getAppliedOutput() * 12.0);
        shooterSim.update(0.02);

        flywheelMotorSim.iterate(
            shooterSim.getAngularVelocityRPM(),
            12.0,
            0.02
        );

        // simple FF-based bang-bang controller if running
        if (isRunning) {
            flywheelController.setSetpoint(
                targetRPM,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0
            );
        } else {
            flywheelController.setSetpoint(
                0.0,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
        }

        Logger.recordOutput(
            "Simulation/Flywheel/velocityRPM",
            flywheelEncoder.getVelocity()
        );

        inputs.velocityRadPerSecond = shooterSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = shooterSim.getInputVoltage();
        inputs.currentAmps = shooterSim.getCurrentDrawAmps();
        inputs.motorsConnected = true;
        // 300 RPM
        if (inputs.velocityRadPerSecond > 10.0 * Math.PI) {
            if (transitionSim.transitionFuel()) {
                Pose2d drivePose = driveSim.getSimulatedDriveTrainPose();

                // shoot a fuel
                // exit rpm should be 3850
                RebuiltFuelOnFly shotFuel = new RebuiltFuelOnFly(
                    drivePose.getTranslation(),
                    new Translation2d(0.25, 0.0),
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    drivePose.getRotation(),
                    Inches.of(20),
                    // ≈10 m/s @ 3000 RPM
                    MetersPerSecond.of(
                        (inputs.velocityRadPerSecond / Math.PI / 100) * 10.0
                    ),
                    Degrees.of(hoodSim.getAngleDeg())
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
                            "Simulation/Shooter/SuccessfulShot",
                            locations.toArray(Pose3d[]::new)
                        );
                    },
                    locations -> {
                        Logger.recordOutput(
                            "Simulation/Shooter/UnsuccessfulShot",
                            locations.toArray(Pose3d[]::new)
                        );
                    }
                );

                Logger.recordOutput(
                    "Simulation/Shooter/ShotDistance",
                    drivePose
                        .getTranslation()
                        .getDistance(new Translation2d(4.5974, 4.03536))
                );

                if (transitionSim.isUnlimited()) {
                    shotFuel.disableBecomesGamePieceOnFieldAfterTouchGround();
                } else {
                    shotFuel.enableBecomesGamePieceOnFieldAfterTouchGround();
                }

                SimulatedArena.getInstance().addGamePieceProjectile(shotFuel);

                shooterSim.setAngularVelocity(
                    shooterSim.getAngularVelocityRadPerSec() * 0.96375
                );
            }
        }
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
