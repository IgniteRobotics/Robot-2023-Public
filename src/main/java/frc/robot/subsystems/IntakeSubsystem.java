// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;

  private DigitalInput cubeBeamBreak = new DigitalInput(1);
  private DigitalInput coneBeamBreak = new DigitalInput(0);

  public static enum GamePiece {
    CUBE, CONE
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    this.intakeMotor = new CANSparkMax(Constants.CANConstants.INTAKE, MotorType.kBrushless);

    this.intakeMotor.setInverted(false);
    this.intakeMotor.setIdleMode(IdleMode.kBrake);
    this.intakeMotor.setSmartCurrentLimit(25); // this line should never ever be modified or removed.
  }

  public void setIntakePercentOutput(double p) {
    intakeMotor.set(MathUtil.clamp(p, -1, 1));
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  /**
   * Returns whether the cone beam break is broken
   * 
   * @return true if beam break is broken, false otherwise
   */
  public boolean getConeBeamBreak() {
    return !coneBeamBreak.get();
  }

  /**
   * Returns whether the cube beam break is broken
   * 
   * @return true if beam break is broken, false otherwise
   */
  public boolean getCubeBeamBreak() {
    return !cubeBeamBreak.get();
  }

  public boolean isBeamBroken() {
    return getConeBeamBreak() || getCubeBeamBreak();
  }

  public CommandBase intakeEffortCommand(Supplier<Double> effortSupplier) {
    return run(() -> {
      setIntakePercentOutput(effortSupplier.get());
    });
  }

  private CommandBase runIntakeCommand(Supplier<Double> delaySupplier, Supplier<Double> effortSupplier) {
    final Timer timer = new Timer();
    final AtomicBoolean stop = new AtomicBoolean(false);

    return new ParallelDeadlineGroup(
        // a small epsilon is added since timer starts at 0.0
        new WaitUntilCommand(() -> (timer.get() > Math.abs(delaySupplier.get()) || stop.get())),
        runEnd(() -> {
          if (!stop.get())
            setIntakePercentOutput(effortSupplier.get());

          if (isBeamBroken()) {
            timer.start();
          }
        }, this::stop)).beforeStarting(() -> {
          // must reset timer and flag before every schedule
          timer.stop();
          timer.reset();
          stop.set(false);

          // do not run the intake at all if a beam break is already broken
          if (isBeamBroken()) {
            stop.set(true);
          }
        });
  }

  public CommandBase runIntakeCurrentPiece(Supplier<Double> effortSupplier) {
    return runEnd(() -> {
      if (getConeBeamBreak()) {
        setIntakePercentOutput(-Math.abs(effortSupplier.get()));
      } else if (getCubeBeamBreak()) {
        setIntakePercentOutput(Math.abs(effortSupplier.get()));
      } else {
        stop();
      }
    }, this::stop);
  }

  /**
   * Create a command for intaking a cone automatically
   * 
   * @return
   */
  public CommandBase runIntakeConeCommand() {
    // to intake a cone, run the intake with negative effort
    return runIntakeCommand(RobotPreferences.coneBeamBreakDelay, () -> -RobotPreferences.intakeEffort.get());
  }

  /**
   * Create a command for intaking a cube automatically
   * 
   * @return
   */
  public CommandBase runIntakeCubeCommand() {
    // to intake a cone, run the intake with negative effort
    return runIntakeCommand(RobotPreferences.cubeBeamBreakDelay, () -> RobotPreferences.intakeEffort.get());
  }

  /**
   * Creates and returns the proper intake command given the game piece to intake
   * 
   * @param gamePiece game piece type to intake
   * @return either intakeConeCommand or intakeCubeCommand
   */
  public CommandBase getIntakeCommand(GamePiece gamePiece) {
    switch (gamePiece) {
      case CUBE:
        return runIntakeCubeCommand();
      case CONE:
        return runIntakeConeCommand();
      default:
        return null;
    }
  }

  /**
   * Create a command for outtaking the current game piece.
   * Direction is dependent on which beam break is broken at the start of the
   * command
   * 
   * @return
   */
  public CommandBase runOuttakeCommand() {
    final AtomicBoolean isCone = new AtomicBoolean();

    return runEnd(() -> {
      if (isCone.get()) {
        double effortMagnitude = Math.abs(RobotPreferences.outtakeEffort.get());
        setIntakePercentOutput(effortMagnitude);
      } else {
        double effortMagnitude = Math.abs(RobotPreferences.outtakeCubeEffort.get());
        setIntakePercentOutput(-effortMagnitude);
      }
    }, this::stop)
        .beforeStarting(() -> {
          isCone.set(getConeBeamBreak());
        });
  }

  public CommandBase runOuttakeCommand(boolean isCone) {
    return runOuttakeCommand(isCone, false);
  }

  public CommandBase runOuttakeCommand(boolean isCone, boolean stop) {
    return runEnd(() -> {
      double effortMagnitude = Math.abs(RobotPreferences.outtakeEffort.get());
      if (isCone) {
        setIntakePercentOutput(effortMagnitude);
      } else {
        setIntakePercentOutput(-effortMagnitude);
      }
    }, () -> {
      if(stop) {
        this.stop();
      }
    });
  }

  /**
   * Outtakes the cone/cube. Once the piece leaves the intake, the outtake runs
   * for an additional delay
   * 
   * @param delaySupplier
   * @return
   */
  public CommandBase runOuttakeCommand(Supplier<Double> delaySupplier) {
    return runOuttakeCommand(delaySupplier, true);
  }

  public CommandBase runOuttakeCommand(Supplier<Double> delaySupplier, boolean stop) {
    return new ParallelDeadlineGroup(new WaitUntilCommand(() -> !isBeamBroken()), runOuttakeCommand())
        .andThen(new ParallelDeadlineGroup(new WaitCommand(delaySupplier.get()), runOuttakeCommand()))
        .andThen(new InstantCommand(() -> {
          if(stop) {
            this.stop();
          }
        }));
  }

  /**
   * Special outtake command for the cone (fix hiccup issue)
   * 
   * It cannot stop in between because that is what causes the hiccup
   */
  public CommandBase runOuttakeConeCommand(Supplier<Double> delaySupplier, boolean stop) {
    return new ParallelDeadlineGroup(new WaitUntilCommand(() -> !isBeamBroken()), runOuttakeCommand(true, false))
        .andThen(new ParallelDeadlineGroup(new WaitCommand(delaySupplier.get()), runOuttakeCommand(true, false)))
        .andThen(new InstantCommand(() -> {
          if(stop) {
            this.stop();
          }
        }));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake/cubeBeamBreak", cubeBeamBreak.get());
    SmartDashboard.putBoolean("intake/coneBeamBreak", coneBeamBreak.get());
    SmartDashboard.putNumber("intake/outputCurrent", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake/temperature", intakeMotor.getMotorTemperature());
  }
}
