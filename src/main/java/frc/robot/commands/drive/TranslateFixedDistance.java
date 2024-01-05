// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TranslateFixedDistance extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier distanceSupplier;
  private PIDController pidController = new PIDController(2.5, 0, 0);

  public TranslateFixedDistance(DriveSubsystem driveSubsystem, DoubleSupplier distanceSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.distanceSupplier = distanceSupplier;

    addRequirements(driveSubsystem);
  }

  public TranslateFixedDistance(DriveSubsystem driveSubsystem, double distance) {
    this(driveSubsystem, () -> distance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(driveSubsystem.getPose().getY() + distanceSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(new Translation2d(0, pidController.calculate(driveSubsystem.getPose().getY())), new Rotation2d(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
