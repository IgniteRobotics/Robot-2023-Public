// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;
import frc.robot.math.AngleMath;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Default drive command for human driving
 */
public class DriveCommand extends CommandBase {
  private DriveSubsystem drive;
  private Joystick joystick;

  private Translation2d translation;
  private Rotation2d rotation;

  public static final double maxRotationVelocityRadPerSec = Math.PI * 1.5;

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  private boolean slowMode = false;

  private PIDController headingController;
  private Rotation2d targetGyroHeading;
  private boolean strafeMode = false;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive, Joystick joystick) {
    addRequirements(drive);

    this.drive = drive;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(!strafeMode) {
      executeNormalDrive();
    } else if(targetGyroHeading != null) {
      executeStrafeDrive();
    }
  }

  private void executeNormalDrive() {
    double rotationPercent = MathUtil.applyDeadband(-joystick.getRawAxis(4), Constants.JOYSTICK_DEADBAND);
    double joystickX = MathUtil.applyDeadband(-joystick.getRawAxis(0), Constants.JOYSTICK_DEADBAND);
    double joystickY = MathUtil.applyDeadband(-joystick.getRawAxis(1), Constants.JOYSTICK_DEADBAND);

    joystickX = xSpeedLimiter.calculate(joystickX);
    joystickY = ySpeedLimiter.calculate(joystickY);
    rotationPercent = rotLimiter.calculate(rotationPercent);

    translation = new Translation2d(joystickY, joystickX).times(5);
    rotation = new Rotation2d(maxRotationVelocityRadPerSec * rotationPercent);

    if(slowMode) {
      translation = translation.times(RobotPreferences.slowModeMultiplier.getValue());
      rotation = rotation.times(RobotPreferences.slowModeMultiplier.getValue());
    }

    drive.setNeutralMode(NeutralMode.Coast);
    if(joystick.getRawButton(XboxController.Button.kLeftStick.value)) {
      drive.drive(translation, rotation, false);
    } else {
      drive.drive(translation, rotation, true);
    }
  }

  private void executeStrafeDrive() {
    Translation2d joystickDeltas = new Translation2d(MathUtil.applyDeadband(-joystick.getRawAxis(0), 0.1), MathUtil.applyDeadband(-joystick.getRawAxis(5), Constants.JOYSTICK_DEADBAND));
    joystickDeltas.rotateBy(drive.getGyro().getYaw());

    headingController.setP(RobotPreferences.driveHeadingKP.getValue());
    headingController.setI(RobotPreferences.driveHeadingKI.getValue());
    headingController.setD(RobotPreferences.driveHeadingKD.getValue());

    Rotation2d rotationVelocity = Rotation2d.fromDegrees(-headingController.calculate(MathUtil.inputModulus(targetGyroHeading.getDegrees() - AngleMath.absoluteAngle(drive.getGyro().getYaw().getDegrees()), -180, 180)));
    
    // no sudden movements: values must be within threshold
    drive.drive(new Translation2d(joystickDeltas.getY() * RobotPreferences.strafeVelocity.getValue(), joystickDeltas.getX() * RobotPreferences.strafeVelocity.getValue()), rotationVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public void setSlowMode(boolean slowMode) {
      this.slowMode = slowMode;
  }

  // public void enableStrafe(Rotation2d targetHeading) {
  //   this.targetGyroHeading = targetHeading;
  //   this.strafeMode = true;
  // }

  // public void disableStrafe() {
  //   this.strafeMode = false;
  // }
}
