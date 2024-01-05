package frc.robot.comm.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class BooleanPreference extends RobotPreference<Boolean> {
    private boolean defaultValue;

    public BooleanPreference(String key) {
        this(key, false);
    }

    public BooleanPreference(String key, boolean defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        Preferences.initBoolean(key, defaultValue);
    }

    public boolean getValue() {
        return Preferences.getBoolean(super.key, defaultValue);
    }

    @Override
    public Boolean get() {
        return getValue();
    }
}
