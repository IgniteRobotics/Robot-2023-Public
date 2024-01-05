package frc.robot.input;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CombinationTrigger extends Trigger {
    private final String label;

    public CombinationTrigger(Trigger t1, Trigger t2, String label) {
        super(() -> t1.getAsBoolean() && t2.getAsBoolean());
        this.label = label;

        whileTrue(new RunCommand(() -> {
            System.out.println(label);
        }));
    }
}
