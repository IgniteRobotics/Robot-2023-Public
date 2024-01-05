// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DynamicPPSwerveControllerCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotPreferences;
import frc.robot.math.AngleMath;
import frc.robot.model.Gyro;
import frc.robot.model.Module;
import frc.robot.subsystems.drive.PhotonCameraWrapper.Side;
import frc.robot.trajectory.Destination;
import frc.robot.trajectory.PathGenerator;

// All arrays representing swerve modules on the robot are by the following convention:
// FL, FR, BL, BR
public class DriveSubsystem extends SubsystemBase {
  private final Module[] modules = new Module[4];
  private final SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];
  private final Gyro gyro;

  private final double maxLinearSpeed = 5;

  private final Field2d field2d;
  private final SwerveDrivePoseEstimator autonomousPoseEstimator;
  private final SwerveDrivePoseEstimator poseEstimator;
  public final PhotonCameraWrapper photonCameraWrapper;
  
  private boolean autonVisionEnabled = true;

  public DriveSubsystem(Pose2d startingOdometry, boolean isSimulation) {
    if (isSimulation) {
      modules[0] = new SimModule();
      modules[1] = new SimModule();
      modules[2] = new SimModule();
      modules[3] = new SimModule();
      gyro = new SimGyro();
    } else {
      modules[0] = new FalconModule(DriveConstants.FL, DriveConstants.wheelRadius);
      modules[1] = new FalconModule(DriveConstants.FR, DriveConstants.wheelRadius);
      modules[2] = new FalconModule(DriveConstants.BL, DriveConstants.wheelRadius);
      modules[3] = new FalconModule(DriveConstants.BR, DriveConstants.wheelRadius);
      gyro = new AHRSGyro(new AHRS(SPI.Port.kMXP));
    }

    field2d = new Field2d();
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.swerveKinematics, gyro.getYaw(),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, startingOdometry, VecBuilder.fill(0.95, 0.95, 0.95), VecBuilder.fill(0.05, 0.05, 0.05));

    autonomousPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.swerveKinematics, gyro.getYaw(),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, startingOdometry);


    currentModuleStates[0] = new SwerveModuleState();
    currentModuleStates[1] = new SwerveModuleState();
    currentModuleStates[2] = new SwerveModuleState();
    currentModuleStates[3] = new SwerveModuleState();

    photonCameraWrapper = new PhotonCameraWrapper();
  }

  public DriveSubsystem(Pose2d startingOdometry) {
    this(startingOdometry, false);
  }

  public DriveSubsystem() {
    this(new Pose2d());
  }

  /**
   * Drive the robot given translation and rotation values.
   * Translation should be given in m/s and rotation in rad/s
   * 
   * @param translationMetersPerSecond Translation2d object representing x/y
   *                                   translation velocity
   * @param rotation                   Rotation2d object representing angular
   *                                   velocity
   */
  public void drive(Translation2d translationMetersPerSecond, Rotation2d rotation, boolean fieldOriented) {
    if (fieldOriented) {
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(translationMetersPerSecond.getX(),
          translationMetersPerSecond.getY(), rotation.getRadians()), gyro.getYaw()));
    } else {
      drive(new ChassisSpeeds(translationMetersPerSecond.getX(), translationMetersPerSecond.getY(),
          rotation.getRadians()));
    }
  }

  public void drive(Translation2d translationMetersPerSecond, Rotation2d rotation) {
    drive(translationMetersPerSecond, rotation, false);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    Pose2d currentPosition = new Pose2d();
    Pose2d robotPoseLookahead = new Pose2d(chassisSpeeds.vxMetersPerSecond * Constants.LOOP_PERIOD,
        chassisSpeeds.vyMetersPerSecond * Constants.LOOP_PERIOD,
        Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD));
    Twist2d twistVelocity = currentPosition.log(robotPoseLookahead);
    drive(
        DriveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(twistVelocity.dx / Constants.LOOP_PERIOD,
            twistVelocity.dy / Constants.LOOP_PERIOD, twistVelocity.dtheta / Constants.LOOP_PERIOD)));
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.drive(chassisSpeeds);
  }

  public void drive(SwerveModuleState[] setpointStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      optimizedStates[i] = SwerveModuleState.optimize(setpointStates[i],
          new Rotation2d(modules[i].getTurnAbsolutePosRad()));

      currentModuleStates[i] = optimizedStates[i];
      if (Math.abs(optimizedStates[i].speedMetersPerSecond) >= 0.1) {
        modules[i].setTurnAngle(optimizedStates[i].angle.getRadians());
      }

      modules[i].setDriveVelocity(optimizedStates[i].speedMetersPerSecond);
    }

    if (!gyro.isReal()) {
      ((SimGyro) gyro).timeStep(180
          * (DriveConstants.swerveKinematics.toChassisSpeeds(setpointStates).omegaRadiansPerSecond / Math.PI) * 0.02);
    }
  }

  public void stop() {
    drive(new Translation2d(), new Rotation2d());
  }

  public void setNeutralMode(NeutralMode mode) {
    for (int i = 0; i < 4; ++i) {
      modules[i].setDriveNeutralMode(mode);
    }
  }

  public Gyro getGyro() {
    return this.gyro;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getYaw(), new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    }, pose);
  }

  public void setPoseAuton(Pose2d pose) {
    autonomousPoseEstimator.resetPosition(gyro.getYaw(), new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    }, pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.swerveKinematics.toChassisSpeeds(currentModuleStates);
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public SwerveDrivePoseEstimator getAutonPoseEstimator() {
    return autonomousPoseEstimator;
  }

  public double getModuleAngle(int i) {
    return modules[i].getTurnAbsolutePosRad();
  }

  public double getModuleAngularVelocity(int i) {
    return modules[i].getDriveVelocityRadPerSec();
  }

  public double getModuleDistanceMeters(int i) {
    return modules[i].getPosition().distanceMeters;
  }

  public double getModuleDistanceTicks(int i) {
    return modules[i].getDriveDistanceTicks();
  }

  public double getModuloYawDegrees() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(gyro.getYaw().getRadians())).getDegrees();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; ++i) {
      modules[i].periodic();
    }

    poseEstimator.update(gyro.getYaw(), new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    });

    if(DriverStation.isAutonomous()) {
      autonomousPoseEstimator.update(gyro.getYaw(), new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    });
    }

    Optional<EstimatedRobotPose> estimatedPoseLeft = photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.LEFT);
    Optional<EstimatedRobotPose> estimatedPoseRight = photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.RIGHT);

    if (estimatedPoseLeft.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseLeft.get();
      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
      if(DriverStation.isAutonomous() && autonVisionEnabled) autonomousPoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    if(estimatedPoseRight.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseRight.get();
      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
      if(DriverStation.isAutonomous() && autonVisionEnabled) autonomousPoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field2d);
  }

  public CommandBase runZeroGyroCommand() {
    return startEnd(() -> {
      this.getGyro().zero();
      this.setPose(this.getPose());
    }, () -> {});
  }

  public CommandBase runStrafeCommand(Supplier<Double> strafeSupplier, Supplier<Double> strafeYSupplier, Supplier<Rotation2d> heading) {
    PIDController headingController = new PIDController(0, 0, 0);

    return runEnd(() -> {
      Pose2d pose = getPose();
      Rotation2d rotationVelocity = Rotation2d.fromDegrees(-headingController.calculate(MathUtil.inputModulus(heading.get().getDegrees() - AngleMath.absoluteAngle(gyro.getYaw().getDegrees()), -180, 180)));
      
      // no sudden movements: values must be within threshold
      if(Math.abs(MathUtil.inputModulus(heading.get().getDegrees() - AngleMath.absoluteAngle(gyro.getYaw().getDegrees()), -180, 180)) < 45) {
        drive(new Translation2d(strafeYSupplier.get() * RobotPreferences.strafeVelocity.getValue(), strafeSupplier.get() * RobotPreferences.strafeVelocity.getValue()), rotationVelocity, false);
      } else {
        drive(new Translation2d(strafeYSupplier.get() * RobotPreferences.strafeVelocity.getValue(), strafeSupplier.get() * RobotPreferences.strafeVelocity.getValue()), new Rotation2d(), false);
      }
    }, this::stop).beforeStarting(() -> {
      headingController.setSetpoint(0);

      headingController.setP(RobotPreferences.driveHeadingKP.getValue());
      headingController.setI(RobotPreferences.driveHeadingKI.getValue());
      headingController.setD(RobotPreferences.driveHeadingKD.getValue());
    });
  }

  /**
   * @deprecated Do not use this method, use the 2-axis strafe command instead. This code is left here for example.
   * 
   * Squares the robot to the specified heading, moves it to the specified x
   * distance, and alows only y movement as long as the command is running
   * 
   * @param xDistance
   * @return
   */
  public CommandBase runStrafeYCommand(Supplier<Double> xPosition, Supplier<Rotation2d> heading,
      Supplier<Double> strafeSupplier, Supplier<Double> strafeYSupplier) {
    PIDController xMovementController = new PIDController(2.5, 0, 0);
    PIDController headingController = new PIDController(0, 0, 0);

    return runEnd(() -> {
      Pose2d pose = getPose();
      Rotation2d rotationVelocity = Rotation2d.fromDegrees(-headingController.calculate(MathUtil.inputModulus(heading.get().getDegrees() - AngleMath.absoluteAngle(gyro.getYaw().getDegrees()), -180, 180)));
      
      xMovementController.setSetpoint(xMovementController.getSetpoint() + MathUtil.applyDeadband(strafeYSupplier.get(), 0.1) * RobotPreferences.strafeVelocity.getValue() * 0.01);
      // no sudden movements: values must be within threshold
      if(Math.abs(MathUtil.inputModulus(heading.get().getDegrees() - AngleMath.absoluteAngle(gyro.getYaw().getDegrees()), -180, 180)) < 45) {
        drive(new Translation2d(-xMovementController.calculate(pose.getX()), strafeSupplier.get() * RobotPreferences.strafeVelocity.getValue()), rotationVelocity, false);
      } else {
        drive(new Translation2d(-xMovementController.calculate(pose.getX()), strafeSupplier.get() * RobotPreferences.strafeVelocity.getValue()), new Rotation2d(), false);
      }
    }, this::stop).beforeStarting(() -> {
      Pose2d pose = getPose();

      if(Math.abs(getPose().getTranslation().getX() - 16.9) < getPose().getTranslation().getX() && Math.abs(pose.getX() - xPosition.get()) < 0.7) {
        xMovementController.setSetpoint(16.9 - xPosition.get());
      } else if(Math.abs(pose.getX() - xPosition.get()) < 0.7) {
        xMovementController.setSetpoint(xPosition.get());
      } else {
        xMovementController.setSetpoint(pose.getX());
      }
      headingController.setSetpoint(0);

      headingController.setP(RobotPreferences.driveHeadingKP.getValue());
      headingController.setI(RobotPreferences.driveHeadingKI.getValue());
      headingController.setD(RobotPreferences.driveHeadingKD.getValue());

      xMovementController.setP(RobotPreferences.drivePositionKP.getValue());
      xMovementController.setI(RobotPreferences.drivePositionKI.getValue());
      xMovementController.setD(RobotPreferences.drivePositionKD.getValue());
    });
  }

  public CommandBase followTrajectoryCommand(Translation2d destination) {
    return new DynamicPPSwerveControllerCommand(this, () -> PathGenerator.getInstance().toDestination(getPose(), destination));
  }

  public CommandBase followTrajectoryCommand(Translation2d destination, Rotation2d rot) {
    return new DynamicPPSwerveControllerCommand(this, () -> PathGenerator.getInstance().toDestination(getPose(), destination, rot));
  }

  public CommandBase followTrajectoryCommand(Translation2d destination, Supplier<Rotation2d> rot) {
    return new DynamicPPSwerveControllerCommand(this, () -> PathGenerator.getInstance().toDestination(getPose(), destination, rot.get()));
  }

  public CommandBase followTrajectoryCommand(Supplier<Translation2d> destination, Supplier<Rotation2d> rot) {
    return new DynamicPPSwerveControllerCommand(this, () -> PathGenerator.getInstance().toDestination(getPose(), destination.get(), rot.get()));
  }

  public CommandBase lockWheelsCommand() {
    return new WaitCommand(2).deadlineWith(run(() -> {
      modules[0].setTurnAngle(Math.PI / 4);
      modules[1].setTurnAngle(Math.PI / 4 + Math.PI / 2);
      modules[2].setTurnAngle(Math.PI / 4 + Math.PI / 2);
      modules[3].setTurnAngle(Math.PI / 4);
    }));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("gyro/yaw", () -> gyro.getYaw().getDegrees(), null);
    builder.addDoubleProperty("gyro/absoluteYaw", () -> AngleMath.absoluteAngle(gyro.getYaw().getDegrees()), null);
    builder.addDoubleProperty("gyro/pitch", gyro::getPitch, null);
    builder.addDoubleProperty("gyro/roll", gyro::getRoll, null);

    for (int i = 0; i < 4; ++i) {
      final int index = i;
      builder.addDoubleProperty(String.format("module/%d/angle", i), () -> getModuleAngle(index), null);
      builder.addDoubleProperty(String.format("module/%d/angularVelocity", i), () -> getModuleAngularVelocity(index),
          null);
      builder.addDoubleProperty(String.format("module/%d/distanceMeters", i), () -> getModuleDistanceMeters(index),
          null);
      builder.addDoubleProperty(String.format("module/%d/distanceTicks", i), () -> getModuleDistanceTicks(index), null);
    }
  }

  public void setAutonVisionEnabled(boolean autonVisionEnabled) {
      this.autonVisionEnabled = autonVisionEnabled;
  }
}
