package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.PoseAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import team.vaevictis.victipath.VictiPathBuilder;

public class Autos {

    public static Command firstTestAuto(
        Drive drive,
        Hopper hopper,
        Shooter shooter
    ) {
        Timer timer = new Timer();
        return Commands.sequence(
            Commands.runOnce(() -> {
                timer.restart();
            }),
            VictiPathBuilder.driveTo(
                new Pose2d(new Translation2d(8.2, 0.5), Rotation2d.kCW_90deg)
            ),
            Commands.deadline(
                new PoseAutoAlign(drive, 8.2, 7.25, -Math.PI / 2),
                hopper.runIntake()
            ),
            VictiPathBuilder.driveTo(
                new Pose2d(new Translation2d(1.75, 6.75), Rotation2d.kZero)
            ),
            new PoseAutoAlign(drive, 1.75, 6.75, -Math.PI / 4),
            Commands.runOnce(() -> {
                drive.stopWithX();
            }),
            Commands.deadline(
                Commands.waitUntil(() -> timer.hasElapsed(20)),
                hopper.runTransition(),
                Commands.idle(drive)
            )
        );
    }

    public static Command secondAuto(
        Drive drive,
        Hopper hopper,
        Shooter shooter
    ) {
        Timer timer = new Timer();
        return Commands.sequence(
            Commands.deadline(
                VictiPathBuilder.driveTo(
                    new Pose2d(new Translation2d(0.625, 6.0), Rotation2d.kZero)
                ),
                hopper.runIntake()
            ),
            new PoseAutoAlign(drive, 1.75, 6.75, -Math.PI / 4),
            Rebuilt.alignToHubStatic(drive),
            Commands.runOnce(() -> {
                drive.stopWithX();
                timer.restart();
            }),
            Commands.deadline(
                Commands.waitUntil(() -> timer.hasElapsed(5)),
                hopper.runTransition(),
                Commands.idle(drive)
            ),
            new PoseAutoAlign(drive, 0.625, 5.125, Math.PI)
        );
    }
}
