// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotPreferences;
import frc.robot.model.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;

public class GyroBalanceCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;

  private Translation2d driveDirection;

  /** Creates a new GyroBalanceCommand. */
  public GyroBalanceCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.gyro = driveSubsystem.getGyro();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double pitch = Math.sin(Math.toRadians(MathUtil.applyDeadband(gyro.getPitch(), 2)));
    double roll = Math.sin(Math.toRadians(MathUtil.applyDeadband(gyro.getRoll(), 2)));
    double magnitude = Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
    driveDirection = new Translation2d(pitch, roll).div(magnitude).times(RobotPreferences.gyroBalanceSpeed.getValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveDirection.getX() + " " + driveDirection.getY());
    driveSubsystem.drive(driveDirection, new Rotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // take the magnitude of the cross product of the vectors
    return Math.abs(gyro.getPitch()) < 8 && Math.abs(gyro.getRoll()) < 8;
  }
}
