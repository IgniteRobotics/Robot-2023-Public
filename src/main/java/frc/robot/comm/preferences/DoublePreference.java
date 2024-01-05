package frc.robot.comm.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class DoublePreference extends RobotPreference<Double> {
    private double defaultValue;

    public DoublePreference(String key) {
        this(key, 0.0);
    }

    public DoublePreference(String key, double defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        Preferences.initDouble(key, defaultValue);
    }

    public double getValue() {
        return Preferences.getDouble(super.key, defaultValue);
    }

    @Override
    public Double get() {
        return getValue();
    }
}
