package frc.robot.subsystems.drive;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants.CameraConstants;

public class PhotonCameraWrapper {
    public PhotonCamera photonCameraLeft;
    public PhotonPoseEstimator photonPoseEstimatorLeft;
    public PhotonCamera photonCameraRight;
    public PhotonPoseEstimator photonPoseEstimatorRight;

    public AprilTagFieldLayout layout;

    public static enum Side {
        LEFT, RIGHT
    }

    public PhotonCameraWrapper() {
        photonCameraLeft = new PhotonCamera(CameraConstants.photonCameraNameLeft);
        photonCameraRight = new PhotonCamera(CameraConstants.photonCameraNameRight);
        
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            e.printStackTrace();
        }

        photonPoseEstimatorLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, photonCameraLeft, CameraConstants.photonCameraTransformLeft);
        photonPoseEstimatorRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, photonCameraRight, CameraConstants.photonCameraTransformRight);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Side side) {
        if(side == Side.RIGHT) {
            photonPoseEstimatorRight.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimatorRight.update();
        } else {
            photonPoseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimatorLeft.update();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        return getEstimatedGlobalPose(prevEstimatedRobotPose, Side.LEFT);
    }
}