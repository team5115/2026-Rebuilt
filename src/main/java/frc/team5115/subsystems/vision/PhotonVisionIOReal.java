package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOReal implements PhotonVisionIO {
    public PhotonVisionIOReal() {}

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        for (Camera camera : Camera.values()) {
            inputs.isConnected[camera.ordinal()] = camera.camera.isConnected();
        }
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults(Camera camera) {
        return camera.camera.getAllUnreadResults();
    }

    @Override
    public Optional<EstimatedRobotPose> updatePose(
            Camera camera, PhotonPipelineResult result, Pose3d referencePose) {
        return camera.poseEstimator.estimateCoprocMultiTagPose(result);
    }

    @Override
    public boolean isCameraConnected(PhotonVisionIOInputs inputs, Camera camera) {
        return inputs.isConnected[camera.ordinal()];
    }
}
