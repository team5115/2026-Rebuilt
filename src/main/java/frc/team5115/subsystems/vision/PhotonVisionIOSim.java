package frc.team5115.subsystems.vision;

import frc.team5115.Constants.VisionConstants;
import frc.team5115.MapleSim;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOSim implements PhotonVisionIO {
    private final VisionSystemSim visionSim;

    public PhotonVisionIOSim() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        for (Camera camera : Camera.values()) {
            camera.cameraSim.enableRawStream(false);
            camera.cameraSim.enableProcessedStream(true);
            camera.cameraSim.enableDrawWireframe(false);
            visionSim.addCamera(camera.cameraSim, camera.robotToCamera);
        }
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        for (Camera camera : Camera.values()) {
            inputs.isConnected[camera.ordinal()] = camera.camera.isConnected();
        }
        visionSim.update(MapleSim.getSwerveSim().getSimulatedDriveTrainPose());
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults(Camera camera) {
        return camera.camera.getAllUnreadResults();
    }
}
