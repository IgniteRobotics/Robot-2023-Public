package frc.robot.comm.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class IntegerPreference extends RobotPreference<Integer> {
    private int defaultValue;

    public IntegerPreference(String key) {
        this(key, 0);
    }

    public IntegerPreference(String key, int defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        Preferences.initInt(key,defaultValue);
    }

    public int getValue() {
        return Preferences.getInt(super.key, defaultValue);
    }

    @Override
    public Integer get() {
        return getValue();
    }
}
