// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Sets the drivetrain gyroscope to zero, preserving the robot's current pose in the process.
 * 
 * Useful during field-oriented drive
 */
public class ZeroGyroscopeCommand extends CommandBase {
  private DriveSubsystem drive;

  /** Creates a new ResetOdometryCommand. */
  public ZeroGyroscopeCommand(DriveSubsystem drive) {
    addRequirements(drive);
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.getGyro().zero();
    drive.setPose(drive.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
