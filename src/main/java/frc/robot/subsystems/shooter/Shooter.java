package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double currentAngleSetpoint = 60;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command running() {
        return runOnce(() -> {
                io.setRPMTarget(3000);
            })
            .andThen(idle());
    }

    public Command idling() {
        return runOnce(() -> {
                io.setRPMTarget(0);
            })
            .andThen(idle());
    }

    public Command aimAt(double angle) {
        return Commands.runOnce(() -> {
            io.setAngle(angle);
            currentAngleSetpoint = angle;
        });
    }

    public Command changeAngle(double amountDegreesPerSecond) {
        return Commands.run(() -> {
            currentAngleSetpoint += amountDegreesPerSecond * 0.02;
            io.setAngle(currentAngleSetpoint);
        });
    }

    public Command feedforwardCharacterization() {
        double RAMP_RATE = 0.2;

        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
                timer.restart();
            })
            .andThen(
                run(() -> {
                    double voltage = RAMP_RATE * timer.get();
                    this.io.setVolts(voltage);
                    velocitySamples.add(this.io.getRPM());
                    voltageSamples.add(voltage);
                })
            )
            .finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS =
                    (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV =
                    (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println(
                    "********** Shooter FF Characterization Results **********"
                );
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            });
    }
}
