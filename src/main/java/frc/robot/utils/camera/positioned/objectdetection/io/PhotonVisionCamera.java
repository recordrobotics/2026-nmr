package frc.robot.utils.camera.positioned.objectdetection.io;

import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionClass;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionResult;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionResult.TargetCorner;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision protocol camera for object detection.
 */
public class PhotonVisionCamera extends ObjectDetectionCamera {

    /**
     * The PhotonCamera instance.
     */
    private final PhotonCamera camera;

    /**
     * Constructs a PhotonVisionCamera with the given name, physical camera, and robot-to-camera transform.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera.
     * @param detectionClassMap The map of detection class IDs to ObjectDetectionClass enum.
     */
    public PhotonVisionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d robotToCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        super(name, physicalCamera, robotToCamera, detectionClassMap);

        camera = new PhotonCamera(name);
    }

    /**
     * Checks if the camera is connected.
     * <p>This is different from being enabled. A camera can be enabled but not connected.
     * @return True if the camera is connected, false otherwise.
     */
    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    /**
     * Sets the pipeline index for the camera.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void recordMatchReplay() {
        // not supported
    }

    /**
     * Makes object detections from the camera.
     * @return A list of object detection results.
     */
    @Override
    public List<ObjectDetectionResult> makeDetections() {
        if (!isConnected()) {
            return List.of();
        }

        return getPhotonVisionResults();
    }

    /**
     * Gets the pose estimates from PhotonVision results.
     * @return A list of camera pose estimates.
     */
    @SuppressWarnings("java:S109")
    private List<ObjectDetectionResult> getPhotonVisionResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return List.of();
        }

        List<ObjectDetectionResult> detections = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            result.getTargets().forEach(target -> {
                List<TargetCorner> corners = target.getMinAreaRectCorners().stream()
                        .map(corner -> new TargetCorner(corner.x, corner.y))
                        .toList();

                if (corners.size() != 4) {
                    // PhotonVision should always return 4 corners for a valid target
                    return;
                }

                detections.add(new ObjectDetectionResult(
                        target.getDetectedObjectClassID(),
                        target.getDetectedObjectConfidence(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getArea(),
                        result.getTimestampSeconds(),
                        corners.get(0),
                        corners.get(1),
                        corners.get(2),
                        corners.get(3)));
            });
        }

        return detections;
    }
}
