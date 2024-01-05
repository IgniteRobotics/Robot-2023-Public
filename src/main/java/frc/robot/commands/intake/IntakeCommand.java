package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.GamePiece;

/**
 * General command for intaking a game piece. User may configure the desired elevator height, arm extension, and wrist angle.
 * 
 * Once the assembly reaches the desired setpoints, the intake subsystem runs the intake command until a piece is intaked (beam break is broken)
 */
public class IntakeCommand extends ParallelDeadlineGroup {
    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem, Supplier<Double> elevatorHeight, Supplier<Double> armExtension,
            Supplier<Double> wristAngle, IntakeSubsystem.GamePiece gamePieceType) {
        super(
            intakeSubsystem.getIntakeCommand(gamePieceType),
            elevatorSubsystem.setPositionCommand(elevatorHeight),
            armSubsystem.setArmCommand(armExtension, wristAngle)
        );

        addRequirements(elevatorSubsystem, armSubsystem, intakeSubsystem);
    }
}
