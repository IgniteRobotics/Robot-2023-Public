package frc.robot.model;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    Rotation2d getYaw();
    double getPitch();
    double getRoll();
    void zero();
    boolean isReal();
    default void setOffset(double d) {
        
    }
}
