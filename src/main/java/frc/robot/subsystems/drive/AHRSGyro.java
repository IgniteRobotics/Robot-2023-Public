package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.model.Gyro;

public class AHRSGyro implements Gyro {
    private final AHRS imu;

    public AHRSGyro(AHRS imu) {
        this.imu = imu;
    }

    @Override
    public Rotation2d getYaw() {
        return imu.getRotation2d().times(-1); // this inversion is a property of the AHRSGyro itself
    }
    
    public void setOffset(double d) {
        imu.setAngleAdjustment(d);
    }

    @Override
    public void zero() {
        imu.setAngleAdjustment(0);
        imu.zeroYaw();
    }

    @Override
    public boolean isReal() {
        return true;
    }

    @Override
    public double getPitch() {
        return imu.getPitch();
    }

    @Override
    public double getRoll() {
        return imu.getRoll();
    }
}
