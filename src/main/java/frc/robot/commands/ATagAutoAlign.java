package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ATagAutoAlign extends Command {

    public static AprilTagFieldLayout apriltagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private Drive drive;
    private Vision vision;

    private int tagID;
    private Pose3d tagPose;
    private double posX;
    private double posZ;
    private double rot;

    private LoggedNetworkNumber driveP = new LoggedNetworkNumber(
        "/Tuning/driveP",
        7.5
    );
    private LoggedNetworkNumber driveD = new LoggedNetworkNumber(
        "/Tuning/driveD",
        0.5
    );
    private LoggedNetworkNumber rotP = new LoggedNetworkNumber(
        "/Tuning/rotP",
        5.0
    );
    private LoggedNetworkNumber rotD = new LoggedNetworkNumber(
        "/Tuning/rotD",
        0.0
    );

    // TODO: Tune this
    private PIDController xController = new PIDController(
        driveP.get(),
        0.0,
        driveD.get()
    );
    private PIDController yController = new PIDController(
        driveP.get(),
        0.0,
        driveD.get()
    );
    private PIDController rotController = new PIDController(
        rotP.get(),
        0.0,
        rotD.get()
    );

    private boolean finished = false;

    public ATagAutoAlign(
        Drive drive,
        Vision vision,
        int tagID,
        double posX,
        double posZ,
        double rot
    ) {
        this.drive = drive;
        this.vision = vision;
        this.tagID = tagID;
        this.tagPose = apriltagLayout.getTagPose(tagID).get();
        this.posX = posX;
        this.posZ = posZ;
        this.rot = rot;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xController = new PIDController(driveP.get(), 0.0, driveD.get());
        yController = new PIDController(driveP.get(), 0.0, driveD.get());
        rotController = new PIDController(rotP.get(), 0.0, rotD.get());
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        finished = false;
    }

    @Override
    public void execute() {
        TargetObservation target = vision.getTarget(0);
        int apriltagID = target.id();
        if (apriltagID != tagID) {
            drive.runVelocity(new ChassisSpeeds()); // don't move if the apriltag isn't in view
            finished = true;
            return;
        }

        // Camera-centric DISTANCE ESTIMATION:
        // tan(cam_A + atag_A) = (atag_H - cam_H) / D
        // tan(cam_A + atag_A) / (atag_H - cam_H) = 1 / D
        // (atag_H - cam_H) / tan(cam_A + atag_A) = D

        double camVertAngle = -robotToCamera0.getRotation().getY();
        double tagVertAngle = target.ty().getRadians();
        double heightDifference = tagPose.getZ() - robotToCamera0.getZ();
        double tangent = Math.tan(camVertAngle + tagVertAngle);
        double distance = heightDifference / tangent; // in meters

        // Robot-centric TAG TRANSLATION
        Translation2d cameraRelativeOffset = new Translation2d(
            distance,
            target.tx().unaryMinus().plus(Rotation2d.kCCW_90deg)
        );
        Translation2d robotRelativeOffset = cameraRelativeOffset
            .rotateBy(robotToCamera0.getRotation().toRotation2d())
            .minus(
                new Translation2d(robotToCamera0.getY(), robotToCamera0.getX())
            );

        // Tag-centric ROTATION
        Rotation2d tagRot = tagPose.getRotation().toRotation2d();
        Rotation2d robotRot = tagRot
            .minus(drive.getRotation())
            .plus(Rotation2d.k180deg);

        // PID Updates
        double driveX = xController.calculate(robotRelativeOffset.getX(), posX);
        double driveY = yController.calculate(robotRelativeOffset.getY(), posZ);
        double driveRot = rotController.calculate(robotRot.getRadians(), rot);

        double driveVel = Math.sqrt(driveX * driveX + driveY * driveY);
        double driveOffset = robotRelativeOffset.getDistance(
            new Translation2d(posX, posZ)
        );
        double rotOffset = robotRot.getRadians();

        if (
            driveVel < 0.05 &&
            driveOffset < 0.05 &&
            driveRot < 0.05 &&
            rotOffset < 0.05
        ) {
            // finished
            finished = true;
        }
        Logger.recordOutput("autoAlign/finished", finished);

        ChassisSpeeds speeds = new ChassisSpeeds(-driveY, driveX, -driveRot);

        Logger.recordOutput("autoAlign/speeds", speeds);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
