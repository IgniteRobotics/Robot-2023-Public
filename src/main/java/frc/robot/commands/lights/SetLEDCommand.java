package frc.robot.commands.lights;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.model.BlinkinState;
import frc.robot.subsystems.LightSubsystem;

public class SetLEDCommand extends CommandBase {

    private final LightSubsystem blinkin;

    private final Supplier<BlinkinState> pattern;
    
    public SetLEDCommand(LightSubsystem blinkin, Supplier<BlinkinState> pattern) {
        this.blinkin = blinkin;
        this.pattern = pattern;

        addRequirements(blinkin);
    }

    @Override
    public void execute() {
        blinkin.setPattern(this.pattern.get());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
