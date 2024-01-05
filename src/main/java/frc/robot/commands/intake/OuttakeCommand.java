package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Special command for outtaking cone high. This class can probably become a subclass of OuttakeCommand if we have time to make those refactoring changes
 */
public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(RobotContainer robot, Supplier<Double> elevatorHeight, Supplier<Double> armExtension, Supplier<Double> wristAngle, Supplier<Boolean> doOuttake, Joystick joystick, boolean cone) {
        super(
            new ParallelDeadlineGroup(
                // bring assembly to position and hold until doOuttake true
                new WaitUntilCommand(() -> doOuttake.get() && robot.elevatorSubsystem.atSetpoint() && robot.armSubsystem.armAtSetpoint() && robot.armSubsystem.wristAtSetpoint()).beforeStarting(new WaitCommand(0.1)),
                robot.elevatorSubsystem.setPositionCommand(elevatorHeight),
                // wait until elevator is at 25000 before starting the wrist in order to avoid it crashing
                robot.armSubsystem.setArmCommand(armExtension, wristAngle).beforeStarting(new WaitUntilCommand(() -> robot.elevatorSubsystem.getPosition() >= 15000 || robot.elevatorSubsystem.getMotionSetpoint() < 15000)),
                robot.intakeSubsystem.runIntakeCurrentPiece(RobotPreferences.intakePassiveEffort),
                robot.driveSubsystem.runStrafeCommand(() -> MathUtil.applyDeadband(joystick.getRawAxis(XboxController.Axis.kLeftX.value), Constants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(joystick.getRawAxis(XboxController.Axis.kRightY.value), Constants.JOYSTICK_DEADBAND), () -> Rotation2d.fromDegrees(180))
            ),
            new ParallelDeadlineGroup(
                // outtake while doOuttake true. do not stop once piece leaves
                cone ? robot.intakeSubsystem.runOuttakeConeCommand(RobotPreferences.midConeOuttakeDelay, false) : robot.intakeSubsystem.runOuttakeCommand(false).withTimeout(0.2),
                robot.elevatorSubsystem.setPositionCommand(elevatorHeight),
                robot.armSubsystem.setArmCommand(armExtension, wristAngle)
            ),
            // retract the arm first
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> robot.armSubsystem.getArmPosition() <= 20000).beforeStarting(new WaitCommand(0.04)),
                robot.elevatorSubsystem.setPositionCommand(elevatorHeight),
                robot.armSubsystem.setArmCommand(() -> 0.0, RobotPreferences.wristZeroPosition),
                robot.intakeSubsystem.runOuttakeCommand(true),
                new DriveCommand(robot. driveSubsystem, joystick) // allow driving once piece leaves
            )
        );

        addRequirements(robot.elevatorSubsystem, robot.armSubsystem, robot.intakeSubsystem);
    }
}
