package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import team.vaevictis.victipath.VictiPathBuilder;

public class RebuiltAutos {

    public static Command depotDip(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(0.75, 6.0, Rotation2d.fromDegrees(-20.0))
                ),
                Commands
                    .deadline(Commands.waitSeconds(0.25), hopper.idled())
                    .andThen(hopper.intaking()),
                shooter.hoodIdled()
            ),
            VictiPathBuilder.driveTo(
                new Pose2d(2.45, 4.85, Rotation2d.fromDegrees(-16.7))
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.parallel(
                shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                hopper.transitioning()
            )
        );
    }

    public static Command onlyEight(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.45, 4.85, Rotation2d.fromDegrees(-16.7))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(10),
                Commands.parallel(
                    shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                    hopper.transitioning(),
                    leds.shooting()
                )
            )
        );
    }

    public static Command onlyEightThenDepot(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.45, 4.85, Rotation2d.fromDegrees(-16.7))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(5),
                Commands.parallel(
                    shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                    hopper.transitioning(),
                    leds.shooting()
                )
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(1.2, 6.0, Rotation2d.fromDegrees(0.0))
                ),
                Commands
                    .deadline(Commands.waitSeconds(0.25), hopper.idled())
                    .andThen(hopper.intaking()),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.45, 4.85, Rotation2d.fromDegrees(-16.7))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(10),
                Commands.parallel(
                    shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                    hopper.transitioning(),
                    leds.shooting()
                )
            )
        );
    }

    public static Command onlyEightRightSide(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(40.0))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(10),
                Commands.parallel(
                    shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                    hopper.transitioning(),
                    leds.shooting()
                )
            )
        );
    }

    public static Command eightToOutpost(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.5, 1.75, Rotation2d.fromDegrees(45.0))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(4),
                shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                hopper.transitioning(),
                leds.shooting()
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(0.75, 0.7, Rotation2d.fromDegrees(0.0))
                ),
                shooter.hoodIdled(),
                hopper.idled(),
                leds.drivingToSpot()
            ),
            Commands.runOnce(() -> drive.stopWithX()),
            Commands.deadline(
                Commands.waitSeconds(2),
                shooter.hoodIdled(),
                hopper.idled()
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.5, 1.75, Rotation2d.fromDegrees(45.0))
                ),
                hopper.idled(),
                shooter.hoodIdled(),
                leds.drivingToSpot()
            ),
            Rebuilt.alignToHubStatic(drive, leds),
            Commands.deadline(
                Commands.waitSeconds(10),
                shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                hopper.transitioning(),
                leds.shooting()
            )
        );
    }

    public static Command centerDip(
        Drive drive,
        Shooter shooter,
        Hopper hopper,
        Led leds
    ) {
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(6.0, 5.5, Rotation2d.fromDegrees(135.0))
                ),
                shooter.hoodIdled(),
                hopper.idled(),
                leds.drivingToSpot()
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(7.650, 4.5, Rotation2d.kCCW_90deg)
                ),
                shooter.hoodIdled(),
                hopper.intaking(),
                leds.drivingToSpot()
            ),
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(2.5, 5.5, Rotation2d.fromDegrees(45))
                ),
                shooter.hoodIdled(),
                hopper.idled(),
                leds.drivingToSpot()
            ),
            Commands.deadline(
                Rebuilt.alignToHubStatic(drive, leds),
                shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                hopper.idled()
            ),
            Commands.deadline(
                Commands.waitSeconds(10),
                shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                hopper.transitioning(),
                leds.shooting()
            )
        );
    }
}
