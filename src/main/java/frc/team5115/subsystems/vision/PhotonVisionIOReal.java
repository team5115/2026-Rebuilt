package frc.team5115.subsystems.vision;

import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
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
}
