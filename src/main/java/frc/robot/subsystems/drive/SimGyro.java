package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.model.Gyro;

public class SimGyro implements Gyro {
    private double theta = 0;

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(theta); // TODO figure out gyro simulation
    }

    public void timeStep(double dTheta) {
        theta += dTheta;
    }

    @Override
    public void zero() {

    }

    @Override
    public boolean isReal() {
        return false;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }
}
