package frc.robot.subsystems.drive;

import javax.tools.Diagnostic;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConfig;
import frc.robot.constants.DriveConstants;
import frc.robot.math.AngleMath;
import frc.robot.math.SensorProfile;
import frc.robot.model.Module;

public class FalconModule implements Module {
    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final CANCoder turnEncoder;
    private final double wheelRadius;

    public static final double distanceMultiplier = 1.02;

    private static SensorProfile turnMotorProfile = new SensorProfile(2048, DriveConstants.angleGearRatio);
    private static SensorProfile driveMotorProfile = new SensorProfile(2048, DriveConstants.driveGearRatio);

    private static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DriveConstants.driveKs, DriveConstants.driveKv);

    public FalconModule(ModuleIDs moduleIDs, double wheelRadius) {
        this.wheelRadius = wheelRadius;

        turnEncoder = new CANCoder(moduleIDs.cancoderID, "Default Name");
        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(DriveConfig.swerveCanCoderConfig);
        turnEncoder.configMagnetOffset(-moduleIDs.angleOffset);

        turnMotor = new TalonFX(moduleIDs.turnMotorID, "Default Name");
        turnMotor.configFactoryDefault();
        turnMotor.configAllSettings(DriveConfig.swerveAngleFXConfig);

        driveMotor = new TalonFX(moduleIDs.driveMotorID, "Default Name");
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(DriveConfig.swerveDriveFXConfig);

        configure();
    }

    private void configure() {
        turnMotor.setInverted(true);
        driveMotor.setInverted(true);

        turnMotor.config_kP(0, DriveConstants.turnPID.kP);
        turnMotor.config_kI(0, DriveConstants.turnPID.kI);
        turnMotor.config_kD(0, DriveConstants.turnPID.kD);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.config_kP(0, DriveConstants.drivePID.kP);
        driveMotor.config_kI(0, DriveConstants.drivePID.kI);
        driveMotor.config_kD(0, DriveConstants.drivePID.kD);
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {
        double radPerSec = metersPerSecond / wheelRadius;
        driveMotor.set(TalonFXControlMode.Velocity, driveMotorProfile.radPerSecToU(radPerSec), DemandType.ArbitraryFeedForward, feedForward.calculate(metersPerSecond));
    }

    @Override
    public void setTurnAngle(double radians) {
        double delta = turnMotorProfile.radToU(radians - getTurnAbsolutePosRad());
        turnMotor.set(TalonFXControlMode.Position, turnMotor.getSelectedSensorPosition() + AngleMath.angleWrap(delta, turnMotorProfile.unitsPerRevolution));
    }

    @Override
    public double getTurnAbsolutePosRad() {
        return Math.toRadians(turnEncoder.getAbsolutePosition());
    }

    @Override
    public double getDriveVelocityRadPerSec() {
        return driveMotorProfile.uToRadPerSec(driveMotor.getSelectedSensorVelocity());
    }

    public double getTurnRelativePosTicks() {
        return turnMotor.getSelectedSensorPosition();
    }

    @Override
    public double getDriveDistanceRad() {
        return driveMotorProfile.uToRad(driveMotor.getSelectedSensorPosition());
    }

    @Override
    public double getDriveDistanceTicks() {
        return driveMotor.getSelectedSensorPosition();
    }
    
    @Override
    public void setDriveNeutralMode(NeutralMode mode) {
        driveMotor.setNeutralMode(mode);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceRad() * wheelRadius * distanceMultiplier, new Rotation2d(getTurnAbsolutePosRad()));
    }

    public static class ModuleIDs {
        public final int driveMotorID;
        public final int turnMotorID;
        public final int cancoderID;
        public final double angleOffset;
        
        public ModuleIDs(int driveMotorID, int turnMotorID, int cancoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
        }
    }
}
