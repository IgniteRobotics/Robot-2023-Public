// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.comm.SendableAbsoluteEncoder;
import frc.robot.comm.SendableRelativeEncoder;
import frc.robot.comm.SendableTalonEncoder;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;
import frc.robot.math.AngleMath;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX armMotor;
  private final SendableTalonEncoder armEncoder;

  private final CANSparkMax wristMotor;
  private final SparkMaxPIDController wristController;
  private final AbsoluteEncoder wristEncoder;
  private final RelativeEncoder wristRelativeEncoder;

  private static final double WRIST_TOLERANCE = 50;
  private static final double ARM_TOLERANCE = 500;

  private double wristMotionTarget;
  private double armMotionTarget;

  public static class WristConstants {
    private static double kF = 0.002;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    private static double maxVelocity = 1500;
    private static double maxAcceleration = 12000;
  }

  public static class ArmConstants {
    private static double kF = 0.05;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    private static double maxVelocity = 3000;
    private static double maxAcceleration = 3000;
  }

  public ArmSubsystem() {
    this.armMotor = new TalonFX(Constants.CANConstants.ARM);
    this.armMotor.configFactoryDefault();

    this.armEncoder = new SendableTalonEncoder(this.armMotor);

    this.armMotor.setNeutralMode(NeutralMode.Brake);
    this.armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    this.wristMotor = new CANSparkMax(Constants.CANConstants.WRIST, MotorType.kBrushless);
    this.wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.wristRelativeEncoder = wristMotor.getEncoder();

    this.wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    this.wristMotor.setSoftLimit(SoftLimitDirection.kReverse, 150);
    this.wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.wristMotor.setSoftLimit(SoftLimitDirection.kForward, 1125);

    this.wristController = wristMotor.getPIDController();
    this.wristController.setFeedbackDevice(this.wristEncoder);

    configureSparkMax(wristMotor);
    configureCommands();

    configureWristController(wristController);
    configureArmController(armMotor);

    Preferences.initDouble("arm/arm/kF", 0);
    Preferences.initDouble("arm/arm/kP", 0);
    Preferences.initDouble("arm/arm/kI", 0);
    Preferences.initDouble("arm/arm/kD", 0);

    Preferences.initDouble("arm/arm/maxVelocity", 0);
    Preferences.initDouble("arm/arm/maxAcceleration", 0);
  }

  private void configureCommands() {
    SmartDashboard.putData("arm/arm/ResetEncoders", new InstantCommand(this::resetArmEncoder, this));
  }

  private void configureWristController(SparkMaxPIDController controller) {
    controller.setFF(WristConstants.kF);
    controller.setP(WristConstants.kP);
    controller.setI(WristConstants.kI);
    controller.setD(WristConstants.kD);
    controller.setOutputRange(-1, 1);
    
    // RPM
    controller.setSmartMotionMaxVelocity(WristConstants.maxVelocity, 0);
    controller.setSmartMotionMaxAccel(WristConstants.maxAcceleration, 0);
  }

  private void configureArmController(TalonFX motor) {
    // motion magic configuration
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    motor.configNominalOutputForward(0, 20);
    motor.configNominalOutputReverse(0, 20);
    motor.configPeakOutputForward(1, 20);
    motor.configPeakOutputReverse(-1, 20);

    motor.selectProfileSlot(0, 0);
    motor.config_kF(0, ArmConstants.kF, 10);
    motor.config_kP(0, ArmConstants.kP, 10);
    motor.config_kI(0, ArmConstants.kI, 10);
    motor.config_kD(0, ArmConstants.kD, 10);

    motor.configMotionCruiseVelocity(ArmConstants.maxVelocity, 10);
    motor.configMotionAcceleration(ArmConstants.maxAcceleration, 10);

    motor.configReverseSoftLimitThreshold(10);
    motor.configReverseSoftLimitEnable(true);
    motor.configForwardSoftLimitThreshold(72000);
    motor.configForwardSoftLimitEnable(true);
  }

  private void configureSparkMax(CANSparkMax motor) {
    wristMotor.setInverted(false);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.setSmartCurrentLimit(20);
  }

  public void setWristPercentOutput(double p) {
    wristMotor.set(MathUtil.clamp(p, -1, 1));
  }

  public void setWristPosition(double position) {
    IntakeSubsystem intakeSubsystem = RobotContainer.getInstance().intakeSubsystem;
    if(intakeSubsystem.getConeBeamBreak()) {
      setWristPosition(position, 1);
    } else {
      setWristPosition(position, 0);
    }
  }

  private void setWristPosition(double position, int pidSlot) {
    wristController.setReference(position, ControlType.kSmartMotion, pidSlot);
    setWristSetpoint(position);
  }

  public void setWristSetpoint(double wristMotionTarget) {
    this.wristMotionTarget = wristMotionTarget;
  }

  public void setArmPercentOutput(double p) {
    armMotor.set(ControlMode.PercentOutput, MathUtil.clamp(p, -1, 1));
  }

  public void setArmPosition(double position) {
    armMotor.set(ControlMode.MotionMagic, position);
    setArmSetpoint(position);
  }

  public void setArmSetpoint(double armMotionTarget) {
    this.armMotionTarget = armMotionTarget;
  }

  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  public double getWristVelocity() {
    return wristEncoder.getVelocity();
  }

  public double getArmPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  public double getArmVelocity() {
    return armMotor.getSelectedSensorVelocity();
  }

  public void resetArmEncoder() {
    armMotor.setSelectedSensorPosition(0);
  }

  public boolean wristAtSetpoint() {
    if(Robot.isSimulation()) return true;
    return getWristPosition() >= wristMotionTarget - WRIST_TOLERANCE
        && getWristPosition() <= wristMotionTarget + WRIST_TOLERANCE;
  }

  public boolean wristAtSetpoint(double setpoint) {
    if(Robot.isSimulation()) return true;
    return getWristPosition() >= setpoint - WRIST_TOLERANCE
        && getWristPosition() <= setpoint + WRIST_TOLERANCE;
  }

  public boolean armAtSetpoint() {
    if(Robot.isSimulation()) return true;
    return getArmPosition() >= armMotionTarget - ARM_TOLERANCE && getArmPosition() <= armMotionTarget + ARM_TOLERANCE;
  }

  public void stopWristMotor() {
    wristMotor.stopMotor();
  }

  public void stopArmMotor() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stop() {
    stopWristMotor();
    stopArmMotor();
  }

  @Override
  public void periodic() {
    armMotor.config_kF(0, Preferences.getDouble("arm/arm/kF", 0));
    armMotor.config_kP(0, Preferences.getDouble("arm/arm/kP", 0));
    armMotor.config_kI(0, Preferences.getDouble("arm/arm/kI", 0));
    armMotor.config_kD(0, Preferences.getDouble("arm/arm/kD", 0));

    armMotor.configMotionCruiseVelocity(Preferences.getDouble("arm/arm/maxVelocity", 0));
    armMotor.configMotionAcceleration(Preferences.getDouble("arm/arm/maxAcceleration", 0));

    wristController.setFF(RobotPreferences.wristKF.get(), 0);
    wristController.setP(RobotPreferences.wristKP.get(), 0);
    wristController.setI(RobotPreferences.wristKI.get(), 0);
    wristController.setD(RobotPreferences.wristKD.get(), 0);
    wristController.setSmartMotionMaxVelocity(RobotPreferences.wristMaxVelocity.get(), 0);
    wristController.setSmartMotionMaxAccel(RobotPreferences.wristMaxAccel.get(), 0);

    wristController.setFF(RobotPreferences.wristKFCone.get(), 1);
    wristController.setP(RobotPreferences.wristKPCone.get(), 1);
    wristController.setI(RobotPreferences.wristKICone.get(), 1);
    wristController.setD(RobotPreferences.wristKDCone.get(), 1);
    wristController.setSmartMotionMaxVelocity(RobotPreferences.wristMaxVelocityCone.get(), 1);
    wristController.setSmartMotionMaxAccel(RobotPreferences.wristMaxAccelCone.get(), 1);

    wristRelativeEncoder.setPosition(AngleMath.angleWrap(wristEncoder.getPosition(), 8192));
  }

  /**
   * Creates a command to set the position of the arm and the wrist
   * 
   * @param armPosition   position supplier for arm (encoder ticks)
   * @param wristPosition position supplier for wrist (revolutions of neo)
   * @return
   */
  public CommandBase setArmCommand(Supplier<Double> armPosition, Supplier<Double> wristPosition) {
    return runEnd(() -> {
      setArmPosition(armPosition.get());
      setWristPosition(wristPosition.get());
    }, this::stop);
  }

  public CommandBase setArmCommand(double armPosition, double wristPosition) {
    return this.setArmCommand(() -> armPosition, () -> wristPosition);
  }

  /**
   * Creates a command that sets only the position of the arm
   * 
   * @param armPosition position supplier for arm (encoder ticks)
   * @return
   */
  public CommandBase setArmPositionCommand(Supplier<Double> armPosition) {
    return runEnd(() -> {
      setArmPosition(armPosition.get());
    }, this::stop);
  }

  public CommandBase setArmPositionCommand(double armPosition) {
    return this.setArmPositionCommand(() -> armPosition);
  }

  /**
   * Creates a command that sets only the position of the wrist
   * 
   * @param wristPosition position supplier for wrist (revolutions of neo)
   * @return
   */
  public CommandBase setWristPositionCommand(Supplier<Double> wristPosition) {
    return runEnd(() -> {
      setWristPosition(wristPosition.get());
    }, this::stop);
  }

  public CommandBase setWristPositionCommand(double wristPosition) {
    return this.setWristPositionCommand(() -> wristPosition);
  }

  public CommandBase setWristPositionZeroCommand() {
    return setWristPositionCommand(RobotPreferences.wristZeroPosition);
  }

  public CommandBase setWristEffortCommand(Supplier<Double> effort) {
    return runEnd(() -> {
      setWristPercentOutput(effort.get());
    }, this::stop);
  }

  public CommandBase setWristEffortCommand(double effort) {
    return this.setWristEffortCommand(() -> effort);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("wristEncoder/position", wristEncoder::getPosition, null);
    builder.addDoubleProperty("wristEncoder/relativePosition", wristRelativeEncoder::getPosition, null);
    builder.addDoubleProperty("wristEncoder/velocity", wristEncoder::getVelocity, null);

    builder.addDoubleProperty("armEncoder/position", this::getArmPosition, null);
    builder.addDoubleProperty("armEncoder/velocity", this::getArmVelocity, null);
  }
}