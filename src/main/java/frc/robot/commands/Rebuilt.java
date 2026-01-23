package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DefaultDriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Rebuilt {

    private static TrapezoidProfile.State current;
    private static PIDController angleController = new PIDController(
        5.0,
        0.0,
        0.0
    );

    static {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command driveAlignedToHub(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        TrapezoidProfile angleProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(),
                10.0
            )
        );

        return DefaultDriveCommands
            .joystickDriveWithAngularVelocity(
                drive,
                xSupplier,
                ySupplier,
                () -> {
                    boolean isRed =
                        DriverStation.getAlliance().isPresent() &&
                        DriverStation.getAlliance().get() == Alliance.Red;

                    Translation2d hubLocation = isRed
                        ? new Translation2d(11.938, 4.034536)
                        : new Translation2d(4.5974, 4.034536);

                    double toHubRotation = hubLocation
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .getRadians();

                    Rotation2d driveRot = drive.getRotation();

                    TrapezoidProfile.State target = new TrapezoidProfile.State(
                        toHubRotation,
                        0.0
                    );

                    current = angleProfile.calculate(0.02, current, target);

                    double correction = angleController.calculate(
                        driveRot.getRadians(),
                        current.position
                    );

                    Logger.recordOutput(
                        "AlignedToHub/driveRot",
                        driveRot.getRadians()
                    );
                    Logger.recordOutput(
                        "AlignedToHub/resultVel",
                        current.velocity + correction
                    );

                    return current.velocity + correction;
                }
            )
            .beforeStarting(() -> {
                current =
                    new TrapezoidProfile.State(
                        drive.getRotation().getRadians(),
                        0.0
                    );
                angleController.reset();
            });
    }
}
