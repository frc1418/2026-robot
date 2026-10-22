package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.drive.DefaultDriveCommands;
import frc.robot.commands.drive.PoseAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Rebuilt {

    public static Pose2d alliancePose(double x, double y, Rotation2d r) {
        if (Constants.alliance == Alliance.Red) {
            return new Pose2d(
                16.5354 - x,
                8.0518 - y,
                r.plus(Rotation2d.k180deg)
            );
        } else {
            return new Pose2d(x, y, r);
        }
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
    }

    private static double angleLeft = 100000.0;
    private static double initialAngleLeft = 0.0;

    public static Command alignToHubStatic(Drive drive, Led leds) {
        return DefaultDriveCommands
            .joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0,
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

                    Rotation2d targetRotation = hubLocation
                        .minus(currentTranslation)
                        .getAngle()
                        .minus(Rotation2d.fromDegrees(5.0));

                    angleLeft =
                        targetRotation.minus(drive.getRotation()).getRadians();
                    return targetRotation;
                }
            )
            .alongWith(leds.aiming(() -> angleLeft / initialAngleLeft))
            .beforeStarting(() -> {
                boolean isRed =
                    DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red;

                Translation2d hubLocation = isRed
                    ? new Translation2d(11.938, 4.034536)
                    : new Translation2d(4.5974, 4.034536);

                Translation2d currentTranslation = drive
                    .getPose()
                    .getTranslation();

                Rotation2d targetRotation = hubLocation
                    .minus(currentTranslation)
                    .getAngle()
                    .minus(Rotation2d.fromDegrees(5.0));

                initialAngleLeft =
                    targetRotation.minus(drive.getRotation()).getRadians();
                angleLeft = initialAngleLeft;
            })
            .withDeadline(
                Commands.waitUntil(() -> {
                    return Math.abs(angleLeft) < 0.025;
                })
            )
            .andThen(Commands.runOnce(() -> drive.stopWithX()));
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

    public static DoubleSupplier aimFromHubDistance(Drive drive) {
        final InterpolatingDoubleTreeMap aimingLUT =
            new InterpolatingDoubleTreeMap();

        aimingLUT.put(1.5, 0.495);
        aimingLUT.put(1.75, 0.490);
        aimingLUT.put(2.0, 0.480);
        aimingLUT.put(2.25, 0.460); // 0.75m forward
        aimingLUT.put(2.5, 0.415); // 0.65m forward
        aimingLUT.put(2.75, 0.350); // 0.5m forward
        aimingLUT.put(3.0, 0.295); // 0.5m forward
        aimingLUT.put(3.25, 0.255); // 0.25m forward
        aimingLUT.put(3.5, 0.215); // good
        aimingLUT.put(3.75, 0.150); // good
        aimingLUT.put(4.0, 0.130); // good

        aimingLUT.put(4.25, 0.111); // not shooting from this far
        aimingLUT.put(4.5, 0.066); //
        aimingLUT.put(4.75, 0.046); //

        return () -> {
            boolean isRed =
                DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red;

            Translation2d hubLocation = isRed
                ? new Translation2d(11.938, 4.034536)
                : new Translation2d(4.5974, 4.034536);
            double dist = drive
                .getPose()
                .getTranslation()
                .getDistance(hubLocation);
            Logger.recordOutput("ShotDistance", dist);
            return aimingLUT.get(dist);
        };
    }
}
