package frc.robot.model;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Implementations of this interface are responsible for maintaining a swerve module that can quickly change velocity and angle.
 */
public interface Module {
    /**
     * Set the drive velocity of the module
     * @param metersPerSecond Desired velocity in METERS PER SECOND
     */
    void setDriveVelocity(double metersPerSecond);
    /**
     * Set the angle of the module
     * @param radians Desired angle IN RADIANS
     */
    void setTurnAngle(double radians);

    double getTurnAbsolutePosRad();
    double getDriveVelocityRadPerSec();
    default void setDriveNeutralMode(NeutralMode mode) {}
    double getDriveDistanceRad();
    SwerveModulePosition getPosition();
    double getDriveDistanceTicks();

    default void periodic() {}
}