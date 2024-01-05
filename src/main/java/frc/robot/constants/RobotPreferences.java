package frc.robot.constants;

import frc.robot.comm.preferences.BooleanPreference;
import frc.robot.comm.preferences.DoublePreference;

public class RobotPreferences {
    public static final DoublePreference coneBeamBreakDelay = new DoublePreference("intake/coneDelay", 0.05);
    public static final DoublePreference cubeBeamBreakDelay = new DoublePreference("intake/cubeDelay", 0.0);
    public static final DoublePreference wristZeroPosition = new DoublePreference("wrist/zeroPosition", 320);
    public static final DoublePreference intakePassiveEffort = new DoublePreference("wrist/intakePassiveEffort", 0.7);
    
    public static final DoublePreference intakeEffort = new DoublePreference("intake/intakeEffort", 1);
    public static final DoublePreference outtakeEffort = new DoublePreference("intake/outtakeEffort", 1);
    // separate outtake for cube so it doesn't bounce
    public static final DoublePreference outtakeCubeEffort = new DoublePreference("intake/outtakeCubeEffort", 0.5);

    // outtake parameters
    public static final DoublePreference midConeHeight = new DoublePreference("midConeHeight", 180000);
    public static final DoublePreference midConeExtension = new DoublePreference("midConeExtension", 5500);
    public static final DoublePreference midConeWrist = new DoublePreference("midConeWrist", 1050);
    public static final DoublePreference midConeDistance = new DoublePreference("midConeDistance", 2.1);
    public static final DoublePreference midConeOuttakeDelay = new DoublePreference("midConeOuttakeDelay", 0.25);

    public static final DoublePreference highConeHeight = new DoublePreference("highConeHeight", 180000);
    public static final DoublePreference highConeExtension = new DoublePreference("highConeExtension", 40000);
    public static final DoublePreference highConeWrist = new DoublePreference("highConeWrist", 900);
    public static final DoublePreference highConeDistance = new DoublePreference("highConeDistance", 1.91);

    public static final DoublePreference groundCubeHeight = new DoublePreference("groundCubeHeight", 1050.0);
    public static final DoublePreference groundCubeExtension = new DoublePreference("groundCubeExtension", 15648.0);
    public static final DoublePreference groundCubeWrist = new DoublePreference("groundCubeWrist", 780);
    public static final DoublePreference groundCubeDistance = new DoublePreference("groundCubeDistance", 2.2);

    public static final DoublePreference midCubeHeight = new DoublePreference("midCubeHeight", 150000);
    public static final DoublePreference midCubeExtension = new DoublePreference("midCubeExtension", 45000);
    public static final DoublePreference midCubeWrist = new DoublePreference("midCubeWrist", 1050);
    public static final DoublePreference midCubeOuttakeDelay = new DoublePreference("midCubeOuttakeDelay", 0.3);
    public static final DoublePreference midCubeDistance = new DoublePreference("midCubeDistance", 2.1);

    public static final DoublePreference highCubeHeight = new DoublePreference("highCubeHeight", 180000);
    public static final DoublePreference highCubeExtension = new DoublePreference("highCubeExtension", 55000);
    public static final DoublePreference highCubeWrist = new DoublePreference("highCubeWrist", 700);
    public static final DoublePreference highCubeDistance = new DoublePreference("highCubeDistance", 1.91);

    // intake parameters
    public static final DoublePreference intakeHighAngle = new DoublePreference("intakeHighAngle", 650);
    public static final DoublePreference intakeHighHeight = new DoublePreference("intakeHighHeight", 76500);

    public static final DoublePreference intakeHighAngleCube = new DoublePreference("intakeHighAngleCube", 650);
    public static final DoublePreference intakeHighHeightCube = new DoublePreference("intakeHighHeightCube", 76500);

    public static final DoublePreference intakeConeGroundAngle = new DoublePreference("intakeConeGroundAngle", 1040);
    public static final DoublePreference intakeConeGroundHeight = new DoublePreference("intakeConeGroundHeight", 26500);
    public static final DoublePreference intakeConeGroundExtend = new DoublePreference("intakeConeGroundExtend", 15648);

    public static final DoublePreference intakeCubeGroundAngle = new DoublePreference("intakeCubeGroundAngle", 940);
    public static final DoublePreference intakeCubeGroundHeight = new DoublePreference("intakeCubeGroundHeight", 0.0);
    public static final DoublePreference intakeCubeGroundExtend = new DoublePreference("intakeCubeGroundExtend", 15648);

    public static final DoublePreference wristKF = new DoublePreference("wrist/kF", 0.000002);
    public static final DoublePreference wristKP = new DoublePreference("wrist/kP", 0.000002);
    public static final DoublePreference wristKI = new DoublePreference("wrist/kI", 0);
    public static final DoublePreference wristKD = new DoublePreference("wrist/kD", 0.000008);
    public static final DoublePreference wristMaxVelocity = new DoublePreference("wrist/maxVelocity", 120000);
    public static final DoublePreference wristMaxAccel = new DoublePreference("wrist/maxAccel", 120000);

    public static final DoublePreference wristKFCone = new DoublePreference("wrist/kFCone", 0.000002);
    public static final DoublePreference wristKPCone = new DoublePreference("wrist/kPCone", 0.000002);
    public static final DoublePreference wristKICone = new DoublePreference("wrist/kICone", 0);
    public static final DoublePreference wristKDCone = new DoublePreference("wrist/kDCone", 0.000008);
    public static final DoublePreference wristMaxVelocityCone = new DoublePreference("wrist/maxVelocityCone", 120000);
    public static final DoublePreference wristMaxAccelCone = new DoublePreference("wrist/maxAccelCone", 120000);

    public static final DoublePreference driveHeadingKP = new DoublePreference("drive/heading/kP", 4.5);
    public static final DoublePreference driveHeadingKI = new DoublePreference("drive/heading/kI", 0);
    public static final DoublePreference driveHeadingKD = new DoublePreference("drive/heading/kD", 0);

    public static final DoublePreference drivePositionKP = new DoublePreference("drive/position/kP", 2.8);
    public static final DoublePreference drivePositionKI = new DoublePreference("drive/position/kI", 0);
    public static final DoublePreference drivePositionKD = new DoublePreference("drive/position/kD", 0);

    public static final DoublePreference driveTrajectoryKP = new DoublePreference("drive/trajectory/kP", 5.5);
    public static final DoublePreference driveTrajectoryKPHeading = new DoublePreference("drive/trajectory/heading_kP", 0);

    public static final DoublePreference strafeVelocity = new DoublePreference("drive/strafeVelocity", 0.2);

    public static final DoublePreference slowModeMultiplier = new DoublePreference("drive/slowModeMultiplier", 0.3);

    public static final BooleanPreference manualMode = new BooleanPreference("manualMode", false);

    public static final DoublePreference gyroBalanceSpeed = new DoublePreference("drive/gyroBalanceSpeed", 0);
}