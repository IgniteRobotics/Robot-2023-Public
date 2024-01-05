package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.model.PIDConstants;
import frc.robot.subsystems.drive.FalconModule;
import frc.robot.subsystems.drive.FalconModule.ModuleIDs;

public class DriveConstants {
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(21.5);
    public static final double wheelDiameter = Units.inchesToMeters(3.97436); // TODO
    public static final double wheelRadius = wheelDiameter / 2;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.55 / 1.0); // 6.55:1
    public static final double angleGearRatio = (10.286 / 1.0); // 10.286

    public static final Translation2d[] swerveModuleTransformations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    };

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(swerveModuleTransformations);

    public static final PIDConstants turnPID = new PIDConstants(0.5, 0.0, 0.3);
    public static final PIDConstants drivePID = new PIDConstants(0.05, 0.0, 0);

    public static final double driveKs = 0.25055 / 12;
    public static final double driveKv = 2.2383 / 12;
    public static final double driveKa = 0.29918 / 12;

    public static final FalconModule.ModuleIDs FL = new FalconModule.ModuleIDs(11, 12, 13, 52.14);
    public static final FalconModule.ModuleIDs FR = new FalconModule.ModuleIDs(21, 22, 23, 203.97);
    public static final FalconModule.ModuleIDs BL = new FalconModule.ModuleIDs(31, 32, 33, 136.36);
    public static final FalconModule.ModuleIDs BR = new FalconModule.ModuleIDs(41, 42, 43, 230.9);
    
}