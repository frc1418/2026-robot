// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Rebuilt;
import frc.robot.commands.RebuiltAutos;
import frc.robot.commands.drive.DefaultDriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.intake.IntakeIO;
import frc.robot.subsystems.hopper.intake.IntakeIOReal;
import frc.robot.subsystems.hopper.intake.IntakeIOSim;
import frc.robot.subsystems.hopper.pivot.PivotIO;
import frc.robot.subsystems.hopper.pivot.PivotIOReal;
import frc.robot.subsystems.hopper.pivot.PivotIOSim;
import frc.robot.subsystems.hopper.transition.TransitionIO;
import frc.robot.subsystems.hopper.transition.TransitionIOReal;
import frc.robot.subsystems.hopper.transition.TransitionIOSim;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIO;
import frc.robot.subsystems.led.LedIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import team.vaevictis.victipath.HolonomicPID;
import team.vaevictis.victipath.VictiPathBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final Hopper hopper;
    private final Shooter shooter;
    private final Led leds;
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandJoystick leftJoystick = new CommandJoystick(0);
    private final CommandJoystick rightJoystick = new CommandJoystick(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.drive =
                    new Drive(
                        new GyroIOBoron(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        pose -> {}
                    );

                this.vision =
                    new Vision(
                        drive,
                        new VisionIOLimelight(
                            VisionConstants.camera0Name,
                            drive::getRotation
                        ),
                        new VisionIOLimelight(
                            VisionConstants.camera1Name,
                            drive::getRotation
                        )
                    );

                this.shooter =
                    new Shooter(new FlywheelIOReal(), new HoodIOReal() {});
                this.hopper =
                    new Hopper(
                        new IntakeIOReal(),
                        new PivotIOReal(),
                        new TransitionIOReal()
                    );
                this.leds = new Led(new LedIOReal());
                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                    new SwerveDriveSimulation(
                        DriveConstants.mapleSimConfig,
                        Constants.startingPose
                    );

                // add the simulated drivetrain to the simulation field
                SimulatedArena
                    .getInstance()
                    .addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive =
                    new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose
                    );

                vision =
                    new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                            camera0Name,
                            robotToCamera0,
                            driveSimulation::getSimulatedDriveTrainPose
                        )
                    );

                PivotIOSim pivot = new PivotIOSim();
                IntakeIOSim intake = new IntakeIOSim(driveSimulation, pivot);
                TransitionIOSim transition = new TransitionIOSim(intake, true);

                HoodIOSim hood = new HoodIOSim();
                FlywheelIOSim flywheel = new FlywheelIOSim(
                    driveSimulation,
                    hood,
                    transition
                );

                shooter = new Shooter(flywheel, hood);
                hopper = new Hopper(intake, pivot, transition);
                leds = new Led(new LedIO() {});

                break;
            default:
                // Replayed robot, disable IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        pose -> {}
                    );
                vision =
                    new Vision(drive, new VisionIO() {}, new VisionIO() {});
                shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
                hopper =
                    new Hopper(
                        new IntakeIO() {},
                        new PivotIO() {},
                        new TransitionIO() {}
                    );
                leds = new Led(new LedIO() {});

                break;
        }

        VictiPathBuilder.configure(
            drive,
            () -> {
                boolean isRed =
                    DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red;
                Pose2d pose = drive.getPose();
                if (isRed) {
                    return new Pose2d(
                        16.5354 - pose.getX(),
                        8.0518 - pose.getY(),
                        pose.getRotation().plus(Rotation2d.k180deg)
                    );
                } else {
                    return pose;
                }
            },
            drive::runVelocity,
            new HolonomicPID(2.0, 0.0, 0.0, 5.0, 0.0, 0.0),
            maxSpeedMetersPerSec,
            3.0
        );

        VictiPathBuilder.setLogging((Translation2d[] path) -> {
            boolean isRed =
                DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red;
            Pose2d[] poses = new Pose2d[path.length];
            for (int i = 0; i < path.length; i++) {
                if (isRed) {
                    poses[i] =
                        new Pose2d(
                            16.5354 - path[i].getX(),
                            8.0518 - path[i].getY(),
                            Rotation2d.k180deg
                        );
                } else {
                    poses[i] = new Pose2d(path[i], Rotation2d.kZero);
                }
            }
            Logger.recordOutput("pathFollower/Path", poses);
        });

        // Set up auto routines
        autoChooser =
            new LoggedDashboardChooser<>(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            );

        // Set up SysId routines
        // autoChooser.addOption(
        //     "Drive Wheel Radius Characterization",
        //     DefaultDriveCommands.wheelRadiusCharacterization(drive)
        // );
        // autoChooser.addOption(
        //     "Drive Simple FF Characterization",
        //     DefaultDriveCommands.feedforwardCharacterization(drive)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Forward)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Reverse)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );

        autoChooser.addOption(
            "Only Eight",
            RebuiltAutos.onlyEight(drive, shooter, hopper, leds)
        );
        autoChooser.addOption(
            "OnlyEightThenDepot",
            RebuiltAutos.onlyEightThenDepot(drive, shooter, hopper, leds)
        );
        autoChooser.addOption(
            "OnlyEightRightSide",
            RebuiltAutos.onlyEightRightSide(drive, shooter, hopper, leds)
        );
        autoChooser.addOption(
            "EightToOutpost",
            RebuiltAutos.eightToOutpost(drive, shooter, hopper, leds)
        );
        autoChooser.addOption(
            "CenterDip",
            RebuiltAutos.centerDip(drive, shooter, hopper, leds)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DefaultDriveCommands.joystickDrive(
                drive,
                () -> -leftJoystick.getY(),
                () -> -leftJoystick.getX(),
                () -> -rightJoystick.getX()
            )
        );

        leftJoystick
            .button(1)
            .whileTrue(
                Commands.sequence(
                    Commands.deadline(
                        Rebuilt.alignToHubStatic(drive, leds),
                        shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive))
                    ),
                    Commands.parallel(
                        hopper.transitioning(),
                        shooter.hoodAimed(Rebuilt.aimFromHubDistance(drive)),
                        leds.shiftOrDisabled()
                    )
                )
            );

        // leftJoystick
        //     .button(1)
        //     .whileTrue(
        //         shooter.hoodAimed(() -> {
        //             Logger.recordOutput(
        //                 "HubDistance",
        //                 new Translation2d(11.938, 4.034536)
        //                     .getDistance(drive.getPose().getTranslation())
        //             );
        //             return leftJoystick.getThrottle() * -0.25 + 0.25;
        //         })
        //     );

        leftJoystick.button(2).whileTrue(hopper.intaking());
        leftJoystick.button(3).whileTrue(hopper.transitioning());

        rightJoystick.button(1).whileTrue(hopper.unjamming());

        // rightJoystick.button(1).whileTrue(Rebuilt.moveToTestingPosition(drive));
        // rightJoystick.button(3).onTrue(Rebuilt.resetTestingDistance());
        // rightJoystick.button(4).onTrue(Rebuilt.increaseTestingDistance());

        // leftJoystick
        //     .button(4)
        //     .onTrue(
        //         VictiPathBuilder.driveTo(
        //             new Pose2d(8.230, 4.035, Rotation2d.k180deg)
        //         )
        //     );

        // rightJoystick.button(1).whileTrue(shooter.runShooterFFSysID());

        /*leftJoystick
            .button(3)
            .onTrue(
                Commands.deadline(
                    Commands.waitSeconds(30),
                    shooter.feedforwardCharacterization()
                )
            );*/

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation.getSimulatedDriveTrainPose()
                ) // reset odometry to actual robot pose during simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        Rotation2d.k180deg
                    )
                ); // zero gyro

        rightJoystick.button(2).onTrue(Commands.runOnce(resetGyro));

        SmartDashboard.putNumber("ResetPoseX", 3.5);
        SmartDashboard.putNumber("ResetPoseY", 5.0);
        SmartDashboard.putNumber("ResetPoseTheta", 0.0);
        SmartDashboard.putData(
            Commands
                .runOnce(() -> {
                    double x = SmartDashboard.getNumber("ResetPoseX", 0.0);
                    double y = SmartDashboard.getNumber("ResetPoseY", 0.0);
                    Rotation2d theta = Rotation2d.fromDegrees(
                        SmartDashboard.getNumber("ResetPoseTheta", 0.0)
                    );

                    boolean isRed =
                        DriverStation.getAlliance().isPresent() &&
                        DriverStation.getAlliance().get() == Alliance.Red;

                    if (isRed) {
                        drive.resetOdometry(
                            new Pose2d(
                                16.5354 - x,
                                8.0518 - y,
                                theta.plus(Rotation2d.k180deg)
                            )
                        );
                    } else {
                        drive.resetOdometry(new Pose2d(x, y, theta));
                    }
                })
                .withName("Set Odometry")
                .ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(Constants.startingPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput(
            "FieldSimulation/RobotPosition",
            driveSimulation.getSimulatedDriveTrainPose()
        );
        Logger.recordOutput(
            "FieldSimulation/Fuel",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
        );
        Logger.recordOutput(
            "Odometry/OdoError",
            driveSimulation
                .getSimulatedDriveTrainPose()
                .getTranslation()
                .getDistance(drive.getPose().getTranslation())
        );
    }
}
