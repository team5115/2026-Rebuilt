package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {
        public boolean[] isConnected = new boolean[Camera.values().length];
    }

    public default void updateInputs(PhotonVisionIOInputs inputs) {}

    public default List<PhotonPipelineResult> getAllUnreadResults(Camera camera) {
        return null;
    }

    public default Optional<EstimatedRobotPose> updatePose(
            Camera camera, PhotonPipelineResult result, Pose3d referencePose) {
        return Optional.empty();
    }

    public default boolean isCameraConnected(PhotonVisionIOInputs inputs, Camera camera) {
        return false;
    }
}
