package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVisionIO.PhotonVisionIOInputs;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIOInputs inputs = new PhotonVisionIOInputsAutoLogged();
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public enum Camera {
        LEFT_CAM_POINTS_RIGHT("Black", VisionConstants.LEFT_CAM_TO_ROBOT),
        RIGHT_CAM_POINTS_LEFT("Red", VisionConstants.RIGHT_CAM_TO_ROBOT);

        public final PhotonCameraSim cameraSim;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        public final Transform3d robotToCamera;

        Camera(String name, Transform3d robotToCamera) {
            this(
                    name,
                    VisionConstants.WIDTH_PX,
                    VisionConstants.HEIGHT_PX,
                    VisionConstants.DIAG_FOV_DEGREES,
                    VisionConstants.AVG_ERR_PX,
                    VisionConstants.STD_DEV_ERR_PX,
                    VisionConstants.FPS,
                    VisionConstants.AVG_LATENCY_MS,
                    VisionConstants.STD_DEV_LATENCY_MS,
                    robotToCamera);
        }

        Camera(
                String name,
                int width,
                int height,
                double fovDeg,
                double avgErrPx,
                double stdDevErrPx,
                double fps,
                double avgLatencyMs,
                double stdDevLatencyMs,
                Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            SimCameraProperties cameraProp = new SimCameraProperties();
            camera = new PhotonCamera(name);
            cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fovDeg));
            cameraProp.setFPS(fps);
            cameraProp.setAvgLatencyMs(avgLatencyMs);
            cameraProp.setLatencyStdDevMs(stdDevLatencyMs);
            cameraProp.setCalibError(avgErrPx, stdDevErrPx);
            cameraSim =
                    Constants.currentMode == Constants.Mode.SIM
                            ? new PhotonCameraSim(camera, cameraProp)
                            : null;
            poseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, robotToCamera);
        }
    }

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
    }

    public Optional<String> validateVisionPose(EstimatedRobotPose pose, PhotonPipelineResult result) {
        // Check this first because it is fast
        // Reject measurement if estimated 3d pose says robot is off/under the ground
        final double distanceFromGround = Math.abs(pose.estimatedPose.getTranslation().getZ());
        if (distanceFromGround > VisionConstants.zTranslationThreshold) {
            return Optional.of("OffTheGround," + distanceFromGround);
        }

        // Reject based on existing pose angle
        final double delta =
                pose.estimatedPose
                        .getRotation()
                        .toRotation2d()
                        .minus(drivetrain.getRotation())
                        .getDegrees();
        // Logger.recordOutput("Vision/AngleDelta", delta);
        if (Math.abs(delta) > VisionConstants.angleThreshold) {
            return Optional.of("AngleWrong, delta=" + delta);
        }

        if (pose.targetsUsed.size() == 1) {
            return validateSingleTarget(pose, result);
        } else {
            return validateMultiTarget(pose, result);
        }
    }

    private Optional<String> validateMultiTarget(
            EstimatedRobotPose pose, PhotonPipelineResult result) {
        final double targetCount = pose.targetsUsed.size();
        double totalAmbiguity = 0;
        double totalDistance = 0;
        for (final var target : pose.targetsUsed) {
            totalAmbiguity += target.getPoseAmbiguity();
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        final double averageAmbiguity = totalAmbiguity / targetCount;
        final double averageDistance = totalDistance / targetCount;

        if (averageAmbiguity > VisionConstants.ambiguityThreshold) {
            return Optional.of("MultiAmbiguous, " + averageAmbiguity);
        }

        final double factor = 1 + (targetCount - 2) * VisionConstants.multiTagDistanceFactor;
        final double thresholdDistance = VisionConstants.distanceThreshold * factor;
        if (averageDistance > thresholdDistance) {
            Optional.of(
                    String.format(
                            "MultiFar: dist=%f.3#, threshold=%f.3#", averageDistance, thresholdDistance));
        }

        return Optional.empty();
    }

    private Optional<String> validateSingleTarget(
            EstimatedRobotPose pose, PhotonPipelineResult result) {
        final var target = pose.targetsUsed.get(0);
        // throw away ambiguous
        if (target.poseAmbiguity > VisionConstants.ambiguityThreshold) {
            return Optional.of("SingleAmbiguous");
        }

        // throw away parallel tags
        if (Math.abs(target.getYaw()) < VisionConstants.tagYawThreshold) {
            return Optional.of("SingleYaw, " + target.getYaw() + " degrees");
        }

        // throw away distant tags
        final double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();
        if (distanceToTag > VisionConstants.distanceThreshold) {
            return Optional.of("SingleFar, " + distanceToTag);
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // final Pose3d robotPose = new Pose3d(drivetrain.getPose());
        for (Camera camera : Camera.values()) {
            // final Pose3d referencePose = robotPose.transformBy(camera.robotToCamera);
            final String loggingPrefix = String.format("Vision/%s/", camera.camera.getName());
            final var unread = io.getAllUnreadResults(camera);
            for (final var result : unread) {
                // Update the camera's pose estimator
                // Use this camera's field-relative pose as a reference pose
                final var option = camera.poseEstimator.estimateCoprocMultiTagPose(result);
                if (option.isPresent()) {
                    final EstimatedRobotPose pose = option.get();
                    final var validation = validateVisionPose(pose, result);
                    final boolean success = validation.isEmpty();
                    Logger.recordOutput(loggingPrefix + "ErrorMsg", validation.orElse("Valid"));
                    Logger.recordOutput(loggingPrefix + "Good Measurement?", success);

                    if (success) {
                        Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
                        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                    }
                }
            }
        }
    }

    public boolean areAnyCamerasDisconnected() {
        for (Camera camera : Camera.values()) {
            if (!inputs.isConnected[camera.ordinal()]) {
                return true;
            }
        }
        return false;
    }
}
