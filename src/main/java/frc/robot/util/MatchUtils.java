package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchUtils {

    public static enum ShiftAlliance {
        BLUE,
        RED,
        BOTH,
        NEITHER
    }

    public static record Shift(
        ShiftAlliance alliance,
        double time,
        double shiftLength
    ) {
        public double getProgress() {
            return time / shiftLength;
        }
        public double getTimeLeft() {
            return shiftLength - time;
        }
    }

    public static Shift getCurrentShift() {
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isDisabled()) return new Shift(
            ShiftAlliance.NEITHER,
            0.0,
            1.0
        );
        if (DriverStation.isAutonomous()) return new Shift(
            ShiftAlliance.BOTH,
            matchTime,
            20.0
        );

        boolean blueWonAuto =
            DriverStation.getGameSpecificMessage().charAt(0) == 'B';

        if (matchTime < 5.0) return new Shift(
            ShiftAlliance.BOTH,
            matchTime,
            5.0
        );
        if (blueWonAuto) {
            if (matchTime < 30.0) return new Shift(
                ShiftAlliance.RED,
                matchTime,
                25.0
            );
            if (matchTime < 55.0) return new Shift(
                ShiftAlliance.BLUE,
                matchTime,
                25.0
            );
            if (matchTime < 80.0) return new Shift(
                ShiftAlliance.RED,
                matchTime,
                25.0
            );
            if (matchTime < 105.0) return new Shift(
                ShiftAlliance.BLUE,
                matchTime,
                25.0
            );
        } else {
            if (matchTime < 30.0) return new Shift(
                ShiftAlliance.BLUE,
                matchTime,
                25.0
            );
            if (matchTime < 55.0) return new Shift(
                ShiftAlliance.RED,
                matchTime,
                25.0
            );
            if (matchTime < 80.0) return new Shift(
                ShiftAlliance.BLUE,
                matchTime,
                25.0
            );
            if (matchTime < 105.0) return new Shift(
                ShiftAlliance.RED,
                matchTime,
                25.0
            );
        }
        return new Shift(ShiftAlliance.BOTH, matchTime, 30.0);
    }
}
