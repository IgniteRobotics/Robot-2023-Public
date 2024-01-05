package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class DriveConfig {
    public static CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();
    public static TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    static {
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = false;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;   
        
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    }
}