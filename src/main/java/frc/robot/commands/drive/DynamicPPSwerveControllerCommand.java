package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotPreferences;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DynamicPPSwerveControllerCommand extends CommandBase {
    private PPSwerveControllerCommand command;
    private DriveSubsystem driveSubsystem;
    private Supplier<PathPlannerTrajectory> trajectorySupplier;

    public DynamicPPSwerveControllerCommand(DriveSubsystem driveSubsystem, Supplier<PathPlannerTrajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;
        this.driveSubsystem = driveSubsystem;
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.command = new PPSwerveControllerCommand(
            trajectorySupplier.get(),
            driveSubsystem::getPose,
            DriveConstants.swerveKinematics,
            new PIDController(RobotPreferences.driveTrajectoryKP.getValue(), 0, 0),
            new PIDController(RobotPreferences.driveTrajectoryKP.getValue(), 0, 0),
            new PIDController(RobotPreferences.driveTrajectoryKPHeading.getValue(), 0, 0),
            driveSubsystem::drive,
            false,
            driveSubsystem
        );
        this.command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
