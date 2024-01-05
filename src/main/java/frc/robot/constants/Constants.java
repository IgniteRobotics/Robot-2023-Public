// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double LOOP_PERIOD = 0.02;
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CameraConstants {
    public static final String photonCameraNameLeft = "LEFT";
    // public static final Transform3d photonCameraTransformLeft = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d photonCameraTransformLeft = new Transform3d(new Translation3d(0.1, 0.26, 0.7874), new Rotation3d(0, 20 / 180.0 * Math.PI, 0));
    public static final String photonCameraNameRight = "RIGHT";
    public static final Transform3d photonCameraTransformRight = new Transform3d(new Translation3d(0.1, -0.26, 0.7874), new Rotation3d(0, 20 / 180.0 * Math.PI, 0));
  }

  public static class CANConstants {
    public static final int ELEVATOR = 1;
    public static final int ARM = 2;
    public static final int WRIST = 3;
    public static final int INTAKE = 4;
  }

  public static final int ELEVATOR_POSITION_GROUND = 0;
  public static final int ELEVATOR_POSITION_LOW = 0;
  public static final int ELEVATOR_POSITION_MIDDLE = 0;
  public static final int ELEVATOR_POSITION_HIGH = 0;

  public static final double JOYSTICK_DEADBAND = 0.1;
}
