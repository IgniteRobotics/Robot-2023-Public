package frc.robot.comm.preferences;

import java.util.Collection;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

public abstract class RobotPreference<T> implements Supplier<T> {
    protected String key;

    public RobotPreference(String key) {
        this.key = key;
    }

    public void remove() {
        Preferences.remove(key);
    }

    public static void removeAll() {
        Preferences.removeAll();
    }

    public static Collection<String> getKeys() {
        return Preferences.getKeys();
    }
}
