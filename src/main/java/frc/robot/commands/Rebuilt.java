package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DefaultDriveCommands;
import frc.robot.commands.drive.PoseAutoAlign;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Rebuilt {

    private static TrapezoidProfile.State current;
    private static PIDController angleController = new PIDController(
        1.0,
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
        return DefaultDriveCommands.joystickDriveAtAngle(
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

                Translation2d currentTranslation = drive
                    .getPose()
                    .getTranslation();

                return hubLocation.minus(currentTranslation).getAngle();
            }
        );
        // return DefaultDriveCommands
        //     .joystickDriveWithAngularVelocity(
        //         drive,
        //         xSupplier,
        //         ySupplier,
        //         () -> {
        //             boolean isRed =
        //                 DriverStation.getAlliance().isPresent() &&
        //                 DriverStation.getAlliance().get() == Alliance.Red;

        //             Translation2d hubLocation = isRed
        //                 ? new Translation2d(11.938, 4.034536)
        //                 : new Translation2d(4.5974, 4.034536);

        //             Translation2d currentTranslation = drive
        //                 .getPose()
        //                 .getTranslation();

        //             double angle = hubLocation
        //                 .minus(currentTranslation)
        //                 .getAngle()
        //                 .getRadians();

        //             Rotation2d driveRot = drive.getRotation();

        //             double correction = angleController.calculate(
        //                 driveRot.getRadians(),
        //                 angle
        //             );

        //             Logger.recordOutput(
        //                 "AlignedToHub/driveRot",
        //                 driveRot.getRadians()
        //             );
        //             Logger.recordOutput("AlignedToHub/resultVel", correction);

        //             return correction;
        //         }
        //     )
        //     .beforeStarting(() -> {
        //         current =
        //             new TrapezoidProfile.State(
        //                 drive.getRotation().getRadians(),
        //                 0.0
        //             );
        //         angleController.reset();
        //     });
    }

    public static Command alignToHubStatic(Drive drive) {
        TrapezoidProfile angleProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(),
                10.0
            )
        );

        return Commands
            .run(
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

                    // current.velocity + correction;
                    ChassisSpeeds speeds = new ChassisSpeeds(
                        0.0,
                        0.0,
                        current.velocity + correction
                    );

                    drive.runVelocity(speeds);
                },
                drive
            )
            .until(() -> angleProfile.totalTime() < 0.02)
            .beforeStarting(() -> {
                current =
                    new TrapezoidProfile.State(
                        drive.getRotation().getRadians(),
                        0.0
                    );
                angleController.reset();
            });
    }

    private static double distance = 2.5;
    private static PoseAutoAlign autoAlign;

    public static Command moveToTestingPosition(Drive drive) {
        autoAlign =
            new PoseAutoAlign(drive, 11.938 + distance, 4.034536, Math.PI);
        return autoAlign;
    }

    public static Command resetTestingDistance() {
        return Commands.runOnce(() -> {
            distance = 2.5;
            autoAlign.setNewTarget(11.938 + distance, 4.034536, Math.PI);
            Logger.recordOutput("TestingDist", distance);
        });
    }

    public static Command increaseTestingDistance() {
        return Commands.runOnce(() -> {
            distance += 0.25;
            autoAlign.setNewTarget(11.938 + distance, 4.034536, Math.PI);
            Logger.recordOutput("TestingDist", distance);
        });
    }
}
