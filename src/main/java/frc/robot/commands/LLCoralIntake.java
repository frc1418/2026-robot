package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import org.littletonrobotics.junction.Logger;

public class LLCoralIntake extends Command {

    private Drive drive;
    private Vision limelight;

    private PIDController xController;
    private PIDController yController;

    public LLCoralIntake(Drive drive, Vision limelight) {
        this.drive = drive;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        xController = new PIDController(0.015, 0, 0);
        yController = new PIDController(0.03, 0, 0);

        limelight.setPipeline(0, 1);
    }

    @Override
    public void execute() {
        TargetObservation observation = limelight.getTarget(0);

        Logger.recordOutput("LLCoralIntake/tx", observation.tx());
        Logger.recordOutput("LLCoralIntake/ty", observation.ty());

        double right = xController.calculate(
            observation.tx().getDegrees(),
            0.0
        );
        double forward = yController.calculate(
            observation.ty().getDegrees(),
            -20.0
        );

        Logger.recordOutput("LLCoralIntake/forward", forward);
        Logger.recordOutput("LLCoralIntake/right", right);

        ChassisSpeeds speeds = new ChassisSpeeds(-forward, right, 0.0);

        Logger.recordOutput("LLCoralIntake/speeds", speeds);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setPipeline(0, 0);
    }
}
