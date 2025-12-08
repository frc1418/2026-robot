package frc.robot.commands;

import static frc.robot.Constants.driveD;
import static frc.robot.Constants.driveP;
import static frc.robot.Constants.rotD;
import static frc.robot.Constants.rotP;
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

    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    private boolean finished;

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

        if (
            xController.atSetpoint() &&
            yController.atSetpoint() &&
            rotController.atSetpoint()
        ) {
            // finished
            finished = true;
        }
        Logger.recordOutput("autoAlign/finished", finished);

        ChassisSpeeds speeds = new ChassisSpeeds(-driveY, driveX, -driveRot);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
