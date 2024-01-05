package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.comm.SendableTalonEncoder;
import frc.robot.constants.Constants;
import frc.robot.math.Unit;

public class ElevatorSubsystem extends SubsystemBase {
  private static final int TOLERANCE = 1000;

  private TalonFX elevatorMotor;
  private SendableTalonEncoder elevatorEncoder;
  private NeutralMode motorNeutralMode;

  private final double kF = 0.09857;
  private final double kP = 0.13714;
  private final double kI = 0;
  private final double kD = 0;

  private final int MAX_ACCELERATION = 34500;
  private final int MAX_VELOCITY = 17000;

  private double motionSetpoint;

  // this timer is in place to make the elevator automatically switch to brake mode for safety purposes
  private Timer coastSafetyTimer = new Timer();

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(Constants.CANConstants.ELEVATOR);
    elevatorEncoder = new SendableTalonEncoder(elevatorMotor);

    motorNeutralMode = NeutralMode.Brake;
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    elevatorMotor.setSensorPhase(true);
    elevatorMotor.setInverted(true);

    elevatorMotor.configReverseSoftLimitThreshold(1000);
    elevatorMotor.configReverseSoftLimitEnable(true);

    elevatorMotor.configForwardSoftLimitThreshold(180000);
    elevatorMotor.configForwardSoftLimitEnable(true);

    // motion magic configuration

    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    elevatorMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    elevatorMotor.configNominalOutputForward(0, 20);
    elevatorMotor.configNominalOutputReverse(0, 20);
    elevatorMotor.configPeakOutputForward(1, 20);
    elevatorMotor.configPeakOutputReverse(-1, 20);

    elevatorMotor.selectProfileSlot(0, 0);
    elevatorMotor.config_kF(0, kF, 10);
    elevatorMotor.config_kP(0, kP, 10);
    elevatorMotor.config_kI(0, kI, 10);
    elevatorMotor.config_kD(0, kD, 10);

    elevatorMotor.configMotionCruiseVelocity(MAX_VELOCITY, 10);
    elevatorMotor.configMotionAcceleration(MAX_ACCELERATION, 10);

    configureCommands();

    coastSafetyTimer.start();
  }

  private void configureCommands() {
    SmartDashboard.putData("elevator/ResetEncoders", new InstantCommand(this::resetEncoder, this));
  }

  public void setPercentOutput(double p) {
    elevatorMotor.set(ControlMode.PercentOutput, MathUtil.clamp(p, -1, 1));
  }

  public void setPosition(double position) {
    this.motionSetpoint = position;
    elevatorMotor.set(ControlMode.MotionMagic, position);
  }

  public void setPositionMeters(double meters) {
    setPosition(meters * 112863);
  }

  public void setPositionInches(double inches) {
    setPosition(inches * 2867);
  }

  public boolean isElevatorDoneMoving() {
    return Math.abs(elevatorMotor.getClosedLoopTarget() - elevatorMotor.getSelectedSensorPosition()) <= TOLERANCE;
  }

  public double getPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void stop() {
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return getPosition() >= motionSetpoint - TOLERANCE && getPosition() <= motionSetpoint + 500; 
  }

  public double getMotionSetpoint() {
      return motionSetpoint;
  }

  @Override
  public void periodic() {
    if(Preferences.getBoolean("elevator/coastMode", false)) {
      setCoastMode(true);
    } else {
      setCoastMode(false);
    }

    if(coastSafetyTimer.get() >= 0.04) {
      this.setNeutralMode(NeutralMode.Brake);
    }
  }

  /**
   * Creates a command to set the position of the elevator
   * @param positionSupplier position supplier for elevator
   * @param unit unit for position supplier
   * @return
   */
  public CommandBase setPositionCommand(Supplier<Double> positionSupplier, Unit unit) {
    return runEnd(() -> {
      if(unit == Unit.ENCODER) {
        setPosition(positionSupplier.get());
      } else if(unit == Unit.INCHES) {
        setPositionInches(positionSupplier.get());
      } else if(unit == Unit.METERS) {
        setPositionMeters(positionSupplier.get());
      }
    }, () -> {
      stop();
    });
  }

  public CommandBase setPositionCommand(Supplier<Double> positionSupplier) {
    return this.setPositionCommand(positionSupplier, Unit.ENCODER);
  }

  public CommandBase setPositionCommand(double position, Unit unit) {
    return this.setPositionCommand(() -> position, unit);
  }

  public CommandBase setPositionCommand(double position) {
    return this.setPositionCommand(() -> position, Unit.ENCODER);
  }

  public CommandBase setCoastModeCommand() {
    return runEnd(() -> {
      setCoastMode(true);
    }, () -> {
      setCoastMode(false);
    });
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    this.motorNeutralMode = neutralMode;
    elevatorMotor.setNeutralMode(neutralMode);
  }

  public void setCoastMode(boolean coastMode) {
    if(coastMode) {
      this.setNeutralMode(NeutralMode.Coast);
      coastSafetyTimer.restart();
    } else {
      this.setNeutralMode(NeutralMode.Brake);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("coast", () -> motorNeutralMode == NeutralMode.Coast, this::setCoastMode);
    builder.addDoubleProperty("position", this::getPosition, null);
  }
}