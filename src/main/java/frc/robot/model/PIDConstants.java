package frc.robot.model;

public class PIDConstants {
    public final double kP;
    public final double kI;
    public final double kD;

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
