package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.constants.RobotPreferences;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.GamePiece;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.trajectory.Destination;
import frc.robot.util.PiecePlacement;
import frc.robot.util.PiecePlacement.Location;
import frc.robot.util.PiecePlacement.Location.Grid;

/**
 * This class is responsible for scheduling the proper path and placement command given 
 * the controller combination
 */
public class OuttakeScheduler extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final RobotContainer robotContainer;

    private final Supplier<Boolean> interruptor;

    private final Joystick driver;
    private final Joystick manipulator;

    private Command currentCommand;
    private boolean finished;
    private Runnable cancelAll;

    private PiecePlacement piecePlacement = PiecePlacement.getInstance();
    private Translation2d[] defaultTranslations = PiecePlacement.getDefaultPositions();

    public OuttakeScheduler(
            RobotContainer robotContainer,
            DriveSubsystem driveSubsystem,
            Supplier<Boolean> interruptor,
            Joystick driver,
            Joystick manipulator) {
        this.robotContainer = robotContainer;
        this.driveSubsystem = driveSubsystem;
        this.interruptor = interruptor;

        this.driver = driver;
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        OuttakeCommand highConeOuttake = new OuttakeCommand(robotContainer, RobotPreferences.highConeHeight, RobotPreferences.highConeExtension, RobotPreferences.highConeWrist, () -> driver.getRawButton(XboxController.Button.kA.value), driver, true);
        OuttakeCommand midConeOuttake = new OuttakeCommand(robotContainer, RobotPreferences.midConeHeight, RobotPreferences.midConeExtension, RobotPreferences.midConeWrist, () -> driver.getRawButton(XboxController.Button.kA.value), driver, true);
      
        OuttakeCommand groundCubeOuttake = new OuttakeCommand(robotContainer, RobotPreferences.groundCubeHeight, RobotPreferences.groundCubeExtension, RobotPreferences.groundCubeWrist, () -> true, driver, false);
        OuttakeCommand midCubeOuttake = new OuttakeCommand(robotContainer, RobotPreferences.midCubeHeight, RobotPreferences.midCubeExtension, RobotPreferences.midCubeWrist, () -> true, driver, false);
        OuttakeCommand highCubeOuttake = new OuttakeCommand(robotContainer, RobotPreferences.highCubeHeight, RobotPreferences.highCubeExtension, RobotPreferences.highCubeWrist, () -> true, driver, false);
        
        if (currentCommand != null) {
            CommandScheduler.getInstance().cancel(currentCommand);
        }

        // Get the desired placement position
        Location.Position position = PiecePlacement.getPositionFromJoystick(driver, DriverStation.getAlliance());
        Location.Height height = PiecePlacement.getHeightFromJoystick(manipulator, DriverStation.getAlliance());
        Location.Grid grid = PiecePlacement.getClosestGrid(driveSubsystem);

        Translation2d defaultLocation = defaultTranslations[PiecePlacement.getPlacementIndex(DriverStation.getAlliance(), grid, position, height)];
        // Translation2d estimatedLocation = PiecePlacement.getInstance().getPositionMeasurement(DriverStation.getAlliance(), grid, position, height);

        Translation2d targetLocation = defaultLocation;

        // 1. DRIVE TO POSITION
        CommandBase trajectoryCommand;
        if(DriverStation.getAlliance() == Alliance.Blue) {
            trajectoryCommand = driveSubsystem.followTrajectoryCommand(targetLocation, new Rotation2d(Math.PI));
        } else {
            trajectoryCommand = driveSubsystem.followTrajectoryCommand(targetLocation, new Rotation2d(0));
        }

        // If the robot is too far away from the target, we will not run the path
        if(RobotPreferences.manualMode.getValue() || Math.abs(targetLocation.getX() - driveSubsystem.getPose().getX()) > 3) {
            currentCommand = new InstantCommand(()-> {});
        } else {
            currentCommand = new ParallelRaceGroup(
                new InstantCommand(() -> finished = true).beforeStarting(new WaitUntilCommand(interruptor::get)),
                trajectoryCommand
            );
        }

        // 2. PIECE PLACEMENT AT GRID
        if(height == Location.Height.LOW) {
            currentCommand = currentCommand.andThen(groundCubeOuttake);
        } else if(position == Location.Position.LEFT || position == Location.Position.RIGHT) {
            if(height == Location.Height.HIGH) {
                currentCommand = currentCommand.andThen(highConeOuttake);
            } else if(height == Location.Height.MIDDLE) {
                currentCommand = currentCommand.andThen(midConeOuttake);
            }
        } else if(position == Location.Position.CENTER) {
            if(height == Location.Height.HIGH) {
                currentCommand = currentCommand.andThen(highCubeOuttake);
            } else if(height == Location.Height.MIDDLE) {
                currentCommand = currentCommand.andThen(midCubeOuttake);
            }
        }

        // cancel the placement command at the push of the cancel button
        currentCommand = currentCommand.until(() -> finished);

        // 3. RECORD PLACEMENT POSITION
        currentCommand = currentCommand.andThen(() -> recordPlacement(grid, position, height));

        finished = false;

        // the cancel runnable that cancels all running child command instances of the current outtake scheduler
        // failsafe
        cancelAll = () -> {
            trajectoryCommand.cancel();
            highConeOuttake.cancel();
            midConeOuttake.cancel();
            groundCubeOuttake.cancel();
            midCubeOuttake.cancel();
            highCubeOuttake.cancel();
        };

        CommandScheduler.getInstance().schedule(currentCommand.andThen(() -> finished = true));
    }

    private void recordPlacement(Location.Grid grid, Location.Position position, Location.Height height) {
        // piecePlacement.addPositionMeasurement(driveSubsystem.getPose().getTranslation(), DriverStation.getAlliance(), grid, position, height);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        this.cancelAll.run();
        finished = true;
    }
}
