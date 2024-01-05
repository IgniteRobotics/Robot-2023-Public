package frc.robot.math;

import edu.wpi.first.math.MathUtil;

public class AngleMath {
    public static double angleWrap(double angle, double revSize) {
        revSize = Math.abs(revSize);
        double modAngle = Math.signum(angle) < 0 ? revSize - Math.abs(angle) % revSize : Math.abs(angle) % revSize;
        if(modAngle > revSize / 2) {
            return modAngle - revSize;
        } else {
            return modAngle;
        }
    }

    /**
     * Converts an angle that is in the range [-PI, PI] to one that is in the range [0, 2PI]
     * @param angle
     * @return
     */
    public static double negativePiTo2Pi(double angle) {
        if(angle < 0) {
            return angle + Math.PI * 2;
        } else {
            return angle;
        }
    }

    public static double absoluteAngle(double angle) {
        if(angle < 0) {
            double a = Math.abs(angle);
            a %= 360;
            a *= -1;
            return a + 360;
        } else {
            return angle % 360;
        }
    }
}
