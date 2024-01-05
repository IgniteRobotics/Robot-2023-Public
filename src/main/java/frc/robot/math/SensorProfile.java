package frc.robot.math;

public class SensorProfile {
    public final double unitsPerRevolution;
    public final int velocityRate;

    public SensorProfile(int unitsPerRevolution) {
        this(unitsPerRevolution, 1);
    }

    public SensorProfile(int unitsPerRevolution, double gearRatio) {
        this(unitsPerRevolution, 100, gearRatio);
    }

    public SensorProfile(int unitsPerRevolution, int velocityRate, double gearRatio) {
        this.unitsPerRevolution = unitsPerRevolution * gearRatio;
        this.velocityRate = velocityRate;
    }

    /**
     * Units to Revolutions
     * @param ticks Change in sensor position in sensor units
     * @return Change in sensor position in revolutions
     */
    public double uToRev(double ticks) {
        return (double) ticks / unitsPerRevolution;
    }

    /**
     * Revolutions to Units
     * @param revolutions Change in sensor position in revolutions
     * @return Change in sensor position in sensor units
     */
    public double revToU(double revolutions) {
        return revolutions * unitsPerRevolution;
    }

    /**
     * Units to Radians
     * @param ticks Change in sensor position in sensor units
     * @return Change in sensor position in radians
     */
    public double uToRad(double ticks) {
        return uToRev(ticks) * Math.PI * 2;
    }

    /**
     * Radians to Units
     * @param radians Change in sensor position in radians
     * @return Change in sensor position in sensor units
     */
    public double radToU(double radians) {
        return revToU(radians / Math.PI / 2);
    }

    /**
     * Units to Revolutions Per Second
     * @param ticksPerRate Change in sensor position (ticks) per unit of time (ms)
     * @return Change in sensor position in terms of revolutions per second (RPS)
     */
    public double uToRPS(double ticksPerRate) {
        return ticksPerRate * (1000.0 / velocityRate) * (1.0 / unitsPerRevolution);
    }

    /**
     * Units to Revolutions Per Minute
     * @param ticksPerRate Change in sensor position (ticks) per unit of time (ms)
     * @return Change in sensor position in terms of revolutions per minute (RPM)
     */
    public double uToRPM(double ticksPerRate) {
        return uToRPS(ticksPerRate) * 60;
    }

    /**
     * Revolutions Per Second to Units
     * @param rps Change in sensor position in terms of revolutions per second (RPS)
     * @return Change in sensor position (ticks) per unit of time (ms)
     */
    public double rpsToU(double rps) {
        return (rps * (velocityRate / 1000.0)) * unitsPerRevolution;
    }

    /**
     * Revolutions Per Minute to Units
     * @param rpm Change in sensor position in terms of revolutions per minute (RPM)
     * @return Change in sensor position (ticks) per unit of time (ms)
     */
    public double rpmToU(double rpm) {
        return rpsToU(rpm / 60);
    }

    /**
     * Units to Radians Per Second
     * @param ticksPerRate Change in sensor position (ticks) per unit of time (ms)
     * @return Change in sensor position in terms of radians per second (RPS)
     */
    public double uToRadPerSec(double ticksPerRate) {
        return uToRPS(ticksPerRate) * Math.PI * 2;
    }

    /**
     * Radians Per Second to Units
     * @param radiansPerSecond Change in sensor position in terms of radians per second (RPS)
     * @return Change in sensor position (ticks) per unit of time (ms)
     */
    public double radPerSecToU(double radiansPerSecond) {
        return rpsToU(radiansPerSecond / Math.PI / 2);
    }
}