package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A helper class to make an XBox controller axis act as a button
 */
public class AxisButton extends Trigger {
    private static final double DEFAULT_THRESHOLD = 0.5;

    public AxisButton(Joystick joystick, int axisId, double threshold) {
        super(() -> (boolean) (joystick.getRawAxis(axisId) >= threshold));
    }

    public AxisButton(Joystick joystick, int axisId) {
        this(joystick, axisId, DEFAULT_THRESHOLD);
    }
}
