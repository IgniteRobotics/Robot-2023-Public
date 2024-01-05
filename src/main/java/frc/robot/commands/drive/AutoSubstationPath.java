package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.GamePiece;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoSubstationPath extends SequentialCommandGroup {
  private static Translation2d blueSide = new Translation2d(15.3, 6.1);
  private static Translation2d redSide = new Translation2d(1.22, 6.1);

  private static Translation2d blueSideCube = new Translation2d(15.09, 6.1);
  private static Translation2d redSideCube = new Translation2d(1.13, 6.1);

  public AutoSubstationPath(RobotContainer robot, Joystick joystick, GamePiece gamePiece) {
    super(
      new ParallelDeadlineGroup(
        robot.driveSubsystem.followTrajectoryCommand(() -> getLocation(gamePiece), () -> (DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI))).unless(RobotPreferences.manualMode::get),
        highPieceIntake(robot, gamePiece).beforeStarting(new WaitUntilCommand(() -> Math.abs(getLocation(gamePiece).getX() - robot.driveSubsystem.getPose().getX()) < 2.25))
      ),
      new ParallelDeadlineGroup(
        highPieceIntake(robot, gamePiece),
        robot.driveSubsystem.runStrafeCommand(() -> MathUtil.applyDeadband(joystick.getRawAxis(XboxController.Axis.kLeftX.value), Constants.JOYSTICK_DEADBAND), () -> -MathUtil.applyDeadband(joystick.getRawAxis(XboxController.Axis.kRightY.value), Constants.JOYSTICK_DEADBAND), () -> Rotation2d.fromDegrees(0))
      )
    );
  }

  public static CommandBase highConeIntake(RobotContainer robot) {
    return new IntakeCommand(robot.elevatorSubsystem, robot.armSubsystem, robot.intakeSubsystem, RobotPreferences.intakeHighHeight, () -> 0.0, RobotPreferences.intakeHighAngle, GamePiece.CONE);
  }

  public static CommandBase highCubeIntake(RobotContainer robot) {
    return new IntakeCommand(robot.elevatorSubsystem, robot.armSubsystem, robot.intakeSubsystem, RobotPreferences.intakeHighHeightCube, () -> 0.0, RobotPreferences.intakeHighAngleCube, GamePiece.CUBE);
  }

  public static CommandBase highPieceIntake(RobotContainer robot, GamePiece gamePiece) {
    if(gamePiece == GamePiece.CONE) {
      return highConeIntake(robot);
    } else {
      return highCubeIntake(robot);
    }
  }

  public static Translation2d getLocation(GamePiece gamePiece) {
    if(gamePiece == GamePiece.CONE) {
      if(DriverStation.getAlliance() == Alliance.Red) {
        return redSide;
      } else {
        return blueSide;
      }
    } else {
      if(DriverStation.getAlliance() == Alliance.Red) {
        return redSideCube;
      } else {
        return blueSideCube;
      }
    }
  }
}
