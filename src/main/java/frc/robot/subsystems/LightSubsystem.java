package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.lights.FlashLEDCommand;
import frc.robot.model.BlinkinState;
import frc.robot.subsystems.IntakeSubsystem.GamePiece;
import frc.robot.util.BlinkinController;

public class LightSubsystem extends SubsystemBase {
    private BlinkinController blinkin;
    private BlinkinState pattern;
    
    public LightSubsystem(int blinkinPort) {
        this.blinkin = new BlinkinController(blinkinPort);
    }

    public void turnOff() { setPattern(BlinkinState.Solid_Colors_Black); }
    
    public void setPattern(BlinkinState pattern) {
        this.pattern = pattern;
    }

    private double getSparkValue() {
        if(pattern == null) {
            return BlinkinState.Color_1_Pattern_Breath_Slow.sparkValue;
        }

        return pattern.sparkValue;
    }

    public boolean isActive() { return pattern != BlinkinState.Solid_Colors_Black; }

    @Override
    public void periodic() {
        double powerOutput = getSparkValue();
        blinkin.setPWM(powerOutput);
    }

    public FlashLEDCommand getIntakeSignalCommand(GamePiece piece) {
        // BlinkinState pattern = piece == GamePiece.CONE ? BlinkinState.Solid_Colors_Yellow : BlinkinState.Solid_Colors_Violet;
        // return new FlashLEDCommand(this, pattern, pattern, 100, 3);
        return null;
    }

    public CommandBase getSetLedCommand(Supplier<BlinkinState> pattern) {
        return runOnce(() -> {
            this.setPattern(pattern.get());
        });
    }

    public CommandBase getSetLedCommand(BlinkinState state) {
        return this.getSetLedCommand(() -> state);
    }
}
