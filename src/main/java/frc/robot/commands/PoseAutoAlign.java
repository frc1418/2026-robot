package frc.robot.commands;

import static frc.robot.Constants.driveD;
import static frc.robot.Constants.driveP;
import static frc.robot.Constants.rotD;
import static frc.robot.Constants.rotP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class PoseAutoAlign extends Command {

    private Drive drive;

    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    private double posX;
    private double posZ;
    private double rot;

    private boolean finished;

    public PoseAutoAlign(Drive drive, double posX, double posZ, double rot) {
        this.drive = drive;
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
        Pose2d drivePose = drive.getPose();

        double driveX = xController.calculate(drivePose.getX(), posX);
        double driveY = yController.calculate(drivePose.getY(), posZ);
        double driveRot = rotController.calculate(
            drivePose.getRotation().getRadians(),
            rot
        );

        if (
            xController.atSetpoint() &&
            yController.atSetpoint() &&
            rotController.atSetpoint()
        ) {
            finished = true;
        }
        Logger.recordOutput("autoAlign/finished", finished);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            driveX,
            driveY,
            driveRot,
            drivePose.getRotation()
        );

        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
