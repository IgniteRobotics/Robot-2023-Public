package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.DriveConstants;
import frc.robot.model.Module;

public class SimModule implements Module {
    private double driveVelocity;
    private double turnRelativePosRad = 0.0;
    private double turnAbsolutePosRad = 0; // robot wheels likely start in random orientations

    private double driveDistance = 0.0;

    private double deltaTime = 0.02; // roborio loop time

    public void periodic() {
        driveDistance += driveVelocity * deltaTime;
    }

    public void setDriveVelocity(double velocity) {
        this.driveVelocity = velocity * 20;
    }

    public void setTurnAngle(double angle) {
        turnAbsolutePosRad = angle;
    }

    public double getTurnAbsolutePosRad() {
        return turnAbsolutePosRad;
    }

    public double getDriveVelocityRadPerSec() {
        return driveVelocity;
    }

    public double getDriveDistanceRad() {
        return driveDistance;
    }

    @Override
    public double getDriveDistanceTicks() {
        return 0;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveDistance * DriveConstants.wheelRadius, new Rotation2d(turnAbsolutePosRad));
    }
}
