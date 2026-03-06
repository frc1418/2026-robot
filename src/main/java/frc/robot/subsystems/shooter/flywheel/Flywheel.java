package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

    private FlywheelIO io;
    private FlywheelIOInputsAutoLogged inputs =
        new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;

        this.setDefaultCommand(running());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Robot/Shooter/Flywheel", inputs);
    }

    public Command idled() {
        return runOnce(() -> {
                io.setIdled();
            })
            .andThen(idle())
            .withName("Robot/Shooter/Flywheel/Idled");
    }

    public Command running() {
        return runOnce(() -> {
                io.setRunning();
            })
            .andThen(idle())
            .withName("Robot/Shooter/Flywheel/Running");
    }

    public double getRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(
            inputs.velocityRadPerSecond
        );
    }

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),
            // Allow modules to orient
            run(() -> {
                    io.setVoltageForSysID(0.0);
                })
                .withTimeout(FF_START_DELAY),
            // Start timer
            Commands.runOnce(timer::restart),
            // Accelerate and gather data
            run(() -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    io.setVoltageForSysID(voltage);
                    velocitySamples.add(getRPM());
                    voltageSamples.add(voltage);
                })
                // When cancelled, calculate and print results
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
                        sumX2 +=
                            velocitySamples.get(i) * velocitySamples.get(i);
                    }
                    double kS =
                        (sumY * sumX2 - sumX * sumXY) /
                        (n * sumX2 - sumX * sumX);
                    double kV =
                        (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println(
                        "********** Flywheel FF Characterization Results **********"
                    );
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                })
        );
    }
}
