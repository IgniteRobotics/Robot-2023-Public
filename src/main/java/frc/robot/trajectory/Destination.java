package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum Destination {
    GRID_1_LEFT(0.5, 0.5),
    GRID_1_CENTER(1.071626, 1.071626),
    GRID_1_RIGHT(1.755, 1.75),
    GRID_2_LEFT(2.155, 2.15),
    GRID_2_CENTER(2.7480265, 2.748026),
    GRID_2_RIGHT(3.3, 3.3),
    GRID_3_LEFT(3.8, 3.8),
    GRID_3_CENTER(4.424426, 4.424426),
    GRID_3_RIGHT(5.0, 5.0);
    
    public static final double highBlue = 1.85;
    public static final double midBlue = 2.1;
    public static final double lowBlue = 2;

    public static final double highRed = 14.65;
    public static final double midRed = 14.45;
    public static final double lowRed = 14.5;
    
    public double yRed;
    public double yBlue;

    private Destination(double yRed, double yBlue) {
        this.yRed = yRed;
        this.yBlue = yBlue;
    }

    public double getAllianceSideY(DriverStation.Alliance alliance) {
        if(alliance == DriverStation.Alliance.Blue) {
            return this.yBlue;
        } else {
            return this.yRed;
        }
    }

    public static double getHigh(DriverStation.Alliance alliance) {
        if(alliance == DriverStation.Alliance.Blue) {
            return highBlue;
        } else {
            return highRed;
        }
    }

    public static double getMid(DriverStation.Alliance alliance) {
        if(alliance == DriverStation.Alliance.Blue) {
            return midBlue;
        } else {
            return midRed;
        }
    }

    public static double getLow(DriverStation.Alliance alliance) {
        if(alliance == DriverStation.Alliance.Blue) {
            return lowBlue;
        } else {
            return lowRed;
        }
    }
}
