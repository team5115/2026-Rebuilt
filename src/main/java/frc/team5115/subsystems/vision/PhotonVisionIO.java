package frc.team5115.subsystems.vision;

import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
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
}
