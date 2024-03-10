package frc.robot.subsystems.Vision;

import static frc.robot.Constants.Vision.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;

public class PhotonIO implements VisionIO {
    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    double lastEstTimestamp = 0;

    public PhotonIO(String name) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(kAprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kCameraPose);
    }

    public Pose2d getEstimatedGlobalPose() {
        var visionEst = poseEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst.get().estimatedPose.toPose2d();
    }

    public Pose2d getEstimatedPose() {
        return getEstimatedGlobalPose();
    }

    public double getTimestampSeconds() {
        return camera.getLatestResult().getTimestampSeconds();
    }

    public void updateInputs(VisionIOInputs inputs) {
        inputs.targetX = camera.getLatestResult().getBestTarget().getYaw();
        inputs.targetY = camera.getLatestResult().getBestTarget().getPitch();
        inputs.targetArea = camera.getLatestResult().getBestTarget().getArea();
        inputs.targetID = camera.getLatestResult().getBestTarget().getFiducialId();
    }

    public void periodic() {
        Logger.recordOutput("Vision/" + camera.getName() + "/EstimatedPose", getEstimatedPose());
    }
}
