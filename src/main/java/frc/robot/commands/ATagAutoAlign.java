package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ATagAutoAlign extends Command {

    public static double[] atagHeights = {
        -1.0,
        0.628142,
        7.414259999999999,
        8.031733999999998,
        6.132575999999999,
        1.9098259999999998,
        3.3012379999999997,
        4.0208200000000005,
        4.740402,
        4.740402,
        4.0208200000000005,
        3.3012379999999997,
        0.628142,
        7.414259999999999,
        6.132575999999999,
        1.9098259999999998,
        0.010667999999999999,
        3.3012379999999997,
        4.0208200000000005,
        4.740402,
        4.740402,
        4.0208200000000005,
        3.3012379999999997
    };

    public static double getAtagHeight(int tagId) {
        if (tagId < 1 || tagId > 22) {
            return Double.NaN;
        }
        return atagHeights[tagId];
    }

    private Drive drive;
    private Vision vision;

    private int tagId;
    private double posX;
    private double posZ;
    private double rot;

    public ATagAutoAlign(
        Drive drive,
        Vision vision,
        int tagId,
        double posX,
        double posZ,
        double rot
    ) {
        this.drive = drive;
        this.vision = vision;
        this.tagId = tagId;
        this.posX = posX;
        this.posZ = posZ;
        this.rot = rot;
    }
}
