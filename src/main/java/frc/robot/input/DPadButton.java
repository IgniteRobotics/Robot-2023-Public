package frc.robot.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DPadButton extends Button {
    public DPadButton(GenericHID joystick, Direction direction) {
        super(() -> {
                    int dPadValue = joystick.getPOV();
                    return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                            || (dPadValue == (direction.direction + 315) % 360);
                });
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }
}
