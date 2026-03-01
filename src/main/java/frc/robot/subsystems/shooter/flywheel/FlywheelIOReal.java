package frc.robot.subsystems.shooter.flywheel;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FlywheelIOReal implements FlywheelIO {

    private SparkFlex leftFlywheel = new SparkFlex(9, MotorType.kBrushless);
    private SparkFlex rightFlywheel = new SparkFlex(10, MotorType.kBrushless);

    
    
}
