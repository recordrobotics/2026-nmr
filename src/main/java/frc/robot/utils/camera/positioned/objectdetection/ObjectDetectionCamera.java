package frc.robot.utils.camera.positioned.objectdetection;

import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FieldStateTracker;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.PositionedCamera;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

/**
 * Abstract base class for object detection cameras.
 */
public abstract class ObjectDetectionCamera extends PositionedCamera<ObjectDetectionCamera> {

    /**
     * List of object detection results from last periodic.
     */
    private List<ObjectDetectionResult> cachedDetections = List.of();

    /**
     * The map of detection class IDs to a ObjectDetectionClass.
     */
    private ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap;

    /**
     * Constructs a ObjectDetectionCamera with the given name and physical camera type and default settings.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param detectionClassMap The map of detection class IDs to ObjectDetectionClass enum.
     */
    protected ObjectDetectionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d toCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        super(name, physicalCamera, toCamera);
        this.detectionClassMap = detectionClassMap;
    }

    /**
     * Gets the number of detected objects.
     * @return The number of detected objects.
     */
    public int getNumObjects() {
        return cachedDetections.size();
    }

    /**
     * Gets the detection class map.
     * @return The detection class map.
     */
    public ImmutableSortedMap<Integer, ObjectDetectionClass> getDetectionClassMap() {
        return detectionClassMap;
    }

    /**
     * Sets the detection class map.
     * @param detectionClassMap The detection class map.
     */
    public void setDetectionClassMap(ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        this.detectionClassMap = detectionClassMap;
    }

    /**
     * Makes object detections from the camera.
     * @return A list of object detection results.
     */
    protected abstract List<ObjectDetectionResult> makeDetections();

    /**
     * Periodically updates the cached detections from the camera.
     * <p>This method should be called regularly (e.g. in a main periodic loop).
     */
    @Override
    public void periodic() {
        if (!isEnabled()) {
            cachedDetections = List.of();
            return;
        }

        cachedDetections = makeDetections();
    }

    /**
     * Adds object detection updates to the field state tracker.
     * @param fieldStateTracker The field state tracker.
     */
    public void addObjectDetectionUpdates(FieldStateTracker fieldStateTracker) {
        for (ObjectDetectionResult detection : cachedDetections) {
            ObjectDetectionClass detectionClass = detectionClassMap.get(detection.id());
            if (detectionClass != null) {
                Optional<Pose2d> estimatedPose = estimatePose(detection, detectionClass);
                if (estimatedPose.isPresent()) {
                    fieldStateTracker.addObjectDetectionUpdate(
                            estimatedPose.get(), detectionClass, detection.confidence(), detection.timestamp());
                }
            } else {
                ConsoleLogger.logError("Unknown detection class ID: " + detection.id() + " from camera: " + getName());
            }
        }
    }

    /**
     * Logs camera values.
     * <ul>
     * <li>[NumDetections]: Number of Detections (total across all frames)</li>
     * </ul>
     */
    @Override
    public void logValues() {
        super.logValues();

        String prefix = getPrefix();
        Logger.recordOutput(prefix + "NumDetections", cachedDetections.size());
    }

    /**
     * Estimates the pose of the detected object. Returns empty if pose cannot be estimated.
     * @param detection The object detection result.
     * @param detectionClass The object detection class.
     * @return The estimated pose of the detected object.
     */
    private Optional<Pose2d> estimatePose(ObjectDetectionResult detection, ObjectDetectionClass detectionClass) {

        Optional<Pose2d> fieldToRobot = RobotContainer.poseSensorFusion.getEstimatedPositionAt(detection.timestamp());

        if (fieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        double targetHeightMeters = detectionClass.getHeight();

        Pose3d fieldToCamera = new Pose3d(fieldToRobot.get())
                .transformBy(getRobotToCameraAt(detection.timestamp()).orElse(getRobotToCamera()));

        return Optional.of(new Pose2d(
                estimateFieldToTargetTranslation(
                        targetHeightMeters,
                        Units.degreesToRadians(detection.pitchDegrees()),
                        Rotation2d.fromDegrees(detection.yawDegrees()),
                        fieldToCamera),
                Rotation2d.kZero));
    }

    /**
     * Estimates the field to target translation.
     * @param targetHeightMeters The height of the target in meters.
     * @param targetPitchRadians The pitch of the target in radians.
     * @param targetYaw The yaw of the target.
     * @param fieldToCamera The pose from field to camera.
     * @return The estimated translation from the field to the target.
     */
    private static Translation2d estimateFieldToTargetTranslation(
            double targetHeightMeters, double targetPitchRadians, Rotation2d targetYaw, Pose3d fieldToCamera) {
        double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
                fieldToCamera.getZ(),
                targetHeightMeters,
                fieldToCamera.getRotation().getY(),
                targetPitchRadians);
        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(distanceToTarget, targetYaw);
        return fieldToCamera
                .getTranslation()
                .toTranslation2d()
                .plus(cameraToTarget.rotateBy(fieldToCamera.getRotation().toRotation2d()));
    }
}
