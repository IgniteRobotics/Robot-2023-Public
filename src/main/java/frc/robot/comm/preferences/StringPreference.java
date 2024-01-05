package frc.robot.comm.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class StringPreference extends RobotPreference<String> {
    private String defaultValue;

    public StringPreference(String key) {
        this(key, "");
    }

    public StringPreference(String key, String defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        Preferences.initString(key, defaultValue);
    }

    public String getValue() {
        return Preferences.getString(super.key, defaultValue);
    }

    @Override
    public String get() {
        return getValue();
    }
}
