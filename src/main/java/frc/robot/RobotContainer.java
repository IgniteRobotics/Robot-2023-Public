// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.AutoSubstationPath;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.GyroBalanceCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.OuttakeScheduler;
import frc.robot.commands.lights.FlashLEDCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPreferences;
import frc.robot.input.AxisButton;
import frc.robot.input.CombinationTrigger;
import frc.robot.input.DPadButton;
import frc.robot.model.BlinkinState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.GamePiece;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.trajectory.PathGenerator;
import frc.robot.util.PiecePlacement;

public class RobotContainer {
  private static RobotContainer instance;

  // This class contains input objects for communication from the driver and manipulator to the robot
  // Important note: The simulator incorrectly assigns ids to each axes on the controller.
  private static class Operator {
    private static Joystick driver = new Joystick(0);
    private static Joystick manipulator = new Joystick(1);

    private static DoubleSupplier driver_axisLX = () -> MathUtil.applyDeadband(driver.getRawAxis(0), Constants.JOYSTICK_DEADBAND);
    private static DoubleSupplier driver_axisLY = () -> MathUtil.applyDeadband(-driver.getRawAxis(1), Constants.JOYSTICK_DEADBAND);
    private static DoubleSupplier driver_axisRY = () -> MathUtil.applyDeadband(-driver.getRawAxis(5), Constants.JOYSTICK_DEADBAND);
    private static JoystickButton driver_x = new JoystickButton(driver, XboxController.Button.kX.value);
    private static JoystickButton driver_a = new JoystickButton(driver, XboxController.Button.kA.value);
    private static JoystickButton driver_b = new JoystickButton(driver, XboxController.Button.kB.value);
    private static JoystickButton driver_y = new JoystickButton(driver, XboxController.Button.kY.value);
    private static JoystickButton driver_start = new JoystickButton(driver, XboxController.Button.kStart.value);
    private static JoystickButton driver_back = new JoystickButton(driver, XboxController.Button.kBack.value);
    private static JoystickButton driver_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private static JoystickButton driver_rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private static DPadButton driver_dpadUp = new DPadButton(driver, DPadButton.Direction.UP);
    private static DPadButton driver_dpadDown = new DPadButton(driver, DPadButton.Direction.DOWN);
    private static Trigger driver_leftTrigger = new AxisButton(driver, XboxController.Axis.kLeftTrigger.value, 0.15);
    private static Trigger driver_rightTrigger = new AxisButton(driver, XboxController.Axis.kRightTrigger.value, 0.15);

    private static JoystickButton manip_x = new JoystickButton(manipulator, XboxController.Button.kX.value);
    private static JoystickButton manip_a = new JoystickButton(manipulator, XboxController.Button.kA.value);
    private static JoystickButton manip_b = new JoystickButton(manipulator, XboxController.Button.kB.value);
    private static JoystickButton manip_y = new JoystickButton(manipulator, XboxController.Button.kY.value);
    private static DPadButton manip_dpadUp = new DPadButton(driver, DPadButton.Direction.UP);
    private static DPadButton manip_dpadRight = new DPadButton(driver, DPadButton.Direction.RIGHT);
    private static DPadButton manip_dpadDown = new DPadButton(driver, DPadButton.Direction.DOWN);
    private static DPadButton manip_dpadLeft = new DPadButton(driver, DPadButton.Direction.LEFT);
    private static JoystickButton manip_leftBumper = new JoystickButton(manipulator, XboxController.Button.kLeftBumper.value);
    private static JoystickButton manip_rightBumper = new JoystickButton(manipulator, XboxController.Button.kRightBumper.value);
    private static Trigger manip_rightTrigger = new AxisButton(manipulator, XboxController.Axis.kRightTrigger.value, 0.15);
  }

  // Combination triggers for placement
  private final CombinationTrigger leftLowTrigger = new CombinationTrigger(Operator.driver_x, Operator.manip_a, "LEFT LOW");
  private final CombinationTrigger centerLowTrigger = new CombinationTrigger(Operator.driver_y, Operator.manip_a, "CENTER LOW");
  private final CombinationTrigger rightLowTrigger = new CombinationTrigger(Operator.driver_b, Operator.manip_a, "Right LOW");
  private final CombinationTrigger leftMidTrigger = new CombinationTrigger(Operator.driver_x, Operator.manip_b, "LEFT MID");
  private final CombinationTrigger centerMidTrigger = new CombinationTrigger(Operator.driver_y, Operator.manip_b, "CENTER MID");
  private final CombinationTrigger rightMidTrigger = new CombinationTrigger(Operator.driver_b, Operator.manip_b, "RIGHT MID");
  private final CombinationTrigger leftHighTrigger = new CombinationTrigger(Operator.driver_x, Operator.manip_y, "LEFT HIGH");
  private final CombinationTrigger centerHighTrigger = new CombinationTrigger(Operator.driver_y, Operator.manip_y, "CENTER HIGH");
  private final CombinationTrigger rightHighTrigger = new CombinationTrigger(Operator.driver_b, Operator.manip_y, "RIGHT HIGH");
  private final Trigger outtakeSchedulerTrigger = leftLowTrigger.or(centerLowTrigger).or(rightLowTrigger).or(leftMidTrigger).or(centerMidTrigger).or(rightMidTrigger).or(leftHighTrigger).or(centerHighTrigger).or(rightHighTrigger);

  // SUBSYSTEMS
  public final DriveSubsystem driveSubsystem = new DriveSubsystem(new Pose2d(), Robot.isSimulation());
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LightSubsystem lightSubsystem = new LightSubsystem(0);

  // COMMANDS
  public final DriveCommand driveCommand = new DriveCommand(driveSubsystem, Operator.driver);
  private final CommandBase gyroBalanceAuton = new GyroBalanceCommand(driveSubsystem).andThen(driveSubsystem.lockWheelsCommand());

  // Manual intake commands
  private final CommandBase highConeIntake = new IntakeCommand(elevatorSubsystem, armSubsystem, intakeSubsystem, RobotPreferences.intakeHighHeight, () -> 0.0, RobotPreferences.intakeHighAngle, GamePiece.CONE);
  private final IntakeCommand groundConeIntake = new IntakeCommand(elevatorSubsystem, armSubsystem, intakeSubsystem, RobotPreferences.intakeConeGroundHeight, RobotPreferences.intakeConeGroundExtend, RobotPreferences.intakeConeGroundAngle, GamePiece.CONE);
  private final IntakeCommand groundCubeIntake = new IntakeCommand(elevatorSubsystem, armSubsystem, intakeSubsystem, RobotPreferences.intakeCubeGroundHeight, RobotPreferences.intakeCubeGroundExtend, RobotPreferences.intakeCubeGroundAngle, GamePiece.CUBE);
  
  private final IntakeCommand autoGroundCubeIntake = new IntakeCommand(elevatorSubsystem, armSubsystem, intakeSubsystem, RobotPreferences.intakeCubeGroundHeight, RobotPreferences.intakeCubeGroundExtend, RobotPreferences.intakeCubeGroundAngle, GamePiece.CUBE);

  private final CommandBase manualIntakeConeCommand = intakeSubsystem.intakeEffortCommand(() -> -1.0);

  // Autonomous high substation intake commands
  private final CommandBase autonHighConeIntake = new AutoSubstationPath(this, Operator.manipulator, GamePiece.CONE);
  private final CommandBase autonHighCubeIntake = new AutoSubstationPath(this, Operator.manipulator, GamePiece.CUBE);

  // Scheduler for autonmatic outtake
  private final OuttakeScheduler outtakeScheduler = new OuttakeScheduler(this, driveSubsystem, this::autonomousInterruptor, Operator.driver, Operator.manipulator);

  private final SendableChooser<CommandBase> autonChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    instance = this;
    new RobotPreferences(); // init statics
    PathGenerator.getInstance().initialize(); // initialize PathGenerator
    PiecePlacement.getInstance().initialize();
    DataLogManager.start();

    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setPositionCommand(100));
    driveSubsystem.setDefaultCommand(driveCommand);
    armSubsystem.setDefaultCommand(armSubsystem.setArmCommand(() -> 500.0, RobotPreferences.wristZeroPosition));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCurrentPiece(RobotPreferences.intakePassiveEffort));
    lightSubsystem.setDefaultCommand(lightSubsystem.getSetLedCommand(this::getIntakedBlinkinColor).repeatedly());
    
    SmartDashboard.putData(driveSubsystem);
    SmartDashboard.putData(elevatorSubsystem);
    SmartDashboard.putData(armSubsystem);
    SmartDashboard.putData(outtakeScheduler);
    
    CommandBase autonHighCubeOuttake = new SequentialCommandGroup(
      armSubsystem.setWristPositionCommand(RobotPreferences.wristZeroPosition).until(() -> armSubsystem.wristAtSetpoint(RobotPreferences.wristZeroPosition.getValue())),
      new OuttakeCommand(this, RobotPreferences.highCubeHeight, RobotPreferences.highCubeExtension, RobotPreferences.highCubeWrist, () -> true, Operator.driver, false),
      new WaitUntilCommand(elevatorSubsystem::atSetpoint).beforeStarting(new WaitCommand(0.04)).deadlineWith(elevatorSubsystem.setPositionCommand(1000))
    );

    CommandBase autonMidCubeOuttake = new SequentialCommandGroup(
      armSubsystem.setWristPositionCommand(RobotPreferences.wristZeroPosition).until(() -> armSubsystem.wristAtSetpoint(RobotPreferences.wristZeroPosition.getValue())),
      new OuttakeCommand(this, RobotPreferences.midCubeHeight, RobotPreferences.midCubeExtension, RobotPreferences.midCubeWrist, () -> true, Operator.driver, false).andThen(armSubsystem.setWristPositionCommand(RobotPreferences.wristZeroPosition).withTimeout(1)),
      new WaitUntilCommand(elevatorSubsystem::atSetpoint).beforeStarting(new WaitCommand(0.04)).deadlineWith(elevatorSubsystem.setPositionCommand(1000))
    );
      
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("OuttakeCube", autonHighCubeOuttake);
    eventMap.put("OuttakeCubeMid", autonMidCubeOuttake);
    eventMap.put("DisableVision", new InstantCommand(() -> driveSubsystem.setAutonVisionEnabled(false)));
    eventMap.put("EnableVision", new InstantCommand(() -> driveSubsystem.setAutonVisionEnabled(true)));
    eventMap.put("BeginIntakeCube", new InstantCommand(() -> CommandScheduler.getInstance().schedule(autoGroundCubeIntake)));
    eventMap.put("IntakeCube", autoGroundCubeIntake.withTimeout(2).andThen(armSubsystem.setWristPositionCommand(RobotPreferences.wristZeroPosition).alongWith(intakeSubsystem.intakeEffortCommand(() -> 0.6).withTimeout(2))));
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(() -> driveSubsystem.getAutonPoseEstimator().getEstimatedPosition(), driveSubsystem::setPoseAuton, new PIDConstants(2.5, 0, 0), new PIDConstants(2.5, 0, 0), driveSubsystem::drive, eventMap, true, driveSubsystem);

    PathPlannerTrajectory centerBalance = PathPlanner.loadPath("CenterBalance", new PathConstraints(1, 1));  
    PathPlannerTrajectory oneCubeCenterBalance = PathPlanner.loadPath("OneCubeCenterBalance", new PathConstraints(1, 1));
    PathPlannerTrajectory oneCubeRight = PathPlanner.loadPath("OneCubeRight", new PathConstraints(1, 1));
    PathPlannerTrajectory twoCube = PathPlanner.loadPath("TwoCube", new PathConstraints(2, 2));
    PathPlannerTrajectory twoCubeBump = PathPlanner.loadPath("TwoCubeBump", new PathConstraints(1.5, 1.5));

    autonChooser = new SendableChooser<>();
    autonChooser.addOption("CenterBalance", setGyroBefore(autoBuilder.fullAuto(centerBalance), centerBalance).andThen(gyroBalanceAuton()));
    autonChooser.addOption("OneCubeCenterBalance", setGyroBefore(autoBuilder.fullAuto(oneCubeCenterBalance), oneCubeCenterBalance).andThen(gyroBalanceAuton));
    autonChooser.addOption("OneCubeRight", setGyroBefore(autoBuilder.fullAuto(oneCubeRight), oneCubeRight));
    autonChooser.addOption("TwoCube", setGyroBefore(autoBuilder.fullAuto(twoCube), twoCube));
    // autonChooser.addOption("TwoCubeBump", autoBuilder.followPath(twoCubeBump));
    autonChooser.addOption("None", null);

    SmartDashboard.putData("Autonomous", autonChooser);

    configureBindings();
  }

  private void configureBindings() {
    Operator.driver_start.whileTrue(driveSubsystem.runZeroGyroCommand());
    Operator.driver_back.onTrue(new InstantCommand(() -> outtakeScheduler.cancel()));

    Operator.driver_leftBumper.whileTrue(groundConeIntake);
    Operator.driver_rightBumper.whileTrue(groundCubeIntake);

    Operator.driver_leftTrigger.whileTrue(autonHighConeIntake);
    Operator.driver_rightTrigger.whileTrue(autonHighCubeIntake);

    Operator.manip_leftBumper.toggleOnTrue(new FlashLEDCommand(lightSubsystem, BlinkinState.Solid_Colors_Orange, BlinkinState.Solid_Colors_Orange, 100, 3));
    Operator.manip_rightBumper.toggleOnTrue(new FlashLEDCommand(lightSubsystem, BlinkinState.Solid_Colors_Violet, BlinkinState.Solid_Colors_Violet, 100, 3));

    Operator.manip_rightTrigger.whileTrue(manualIntakeConeCommand);
    
    outtakeSchedulerTrigger.onTrue(outtakeScheduler);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  /**
   * This method returns true when the current autonomous path should be interrupted
   * (i.e. when the driver wishes to regain control of the robot)
   */
  private boolean autonomousInterruptor() {
    double rotationPercent = MathUtil.applyDeadband(-Operator.driver.getRawAxis(4), Constants.JOYSTICK_DEADBAND);
    double joystickX = MathUtil.applyDeadband(-Operator.driver.getRawAxis(0), Constants.JOYSTICK_DEADBAND);
    double joystickY = MathUtil.applyDeadband(-Operator.driver.getRawAxis(1), Constants.JOYSTICK_DEADBAND);

    return rotationPercent > 0 || joystickX > 0 || joystickY > 0;
  }

  /**
   * Return the proper blinkin state for the presently intaked piece
   * @return
   */
  private BlinkinState getIntakedBlinkinColor() {
    if(intakeSubsystem.getConeBeamBreak()) {
      return BlinkinState.Solid_Colors_Orange;
    } else if(intakeSubsystem.getCubeBeamBreak()) {
      return BlinkinState.Solid_Colors_Violet;
    }

    return null;
  }

  public CommandBase setGyroBefore(CommandBase commandBase, PathPlannerTrajectory trajectory) {
    return commandBase.beforeStarting(() -> driveSubsystem.getGyro().setOffset(trajectory.getInitialHolonomicPose().getRotation().getDegrees()));
  }

  public CommandBase gyroBalanceAuton() {
    return new GyroBalanceCommand(driveSubsystem).andThen(driveSubsystem.lockWheelsCommand());
  }
}
