package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.Queue;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOBoron implements GyroIO {

    private final Canandgyro boron = new Canandgyro(boronCanId);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOBoron() {
        yawTimestampQueue =
            SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue =
            SparkOdometryThread.getInstance().registerSignal(boron::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = boron.isConnected();
        inputs.yawPosition = boron.getRotation2d();
        inputs.yawVelocityRadPerSec =
            Units.rotationsToRadians(boron.getAngularVelocityYaw());

        inputs.odometryYawTimestamps =
            yawTimestampQueue
                .stream()
                .mapToDouble((Double value) -> value)
                .toArray();
        inputs.odometryYawPositions =
            yawPositionQueue
                .stream()
                .map((Double value) ->
                    Rotation2d.fromRotations(value).minus(gyroOffset)
                )
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
