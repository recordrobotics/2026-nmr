package frc.robot.utils.camera.positioned.objectdetection.io;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.utils.ContainerUtils;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionClass;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionResult;
import frc.robot.utils.field.FieldIntersection;
import frc.robot.utils.field.FieldIntersection.FieldIntersectionOptions;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.SortedMap;
import java.util.TreeMap;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonUtils;

public class MapleSimCamera extends ObjectDetectionCamera {

    private static final double NETWORK_LATENCY_MS = 7;
    private static final double NETWORK_LATENCY_STD_DEV_MS = 2;

    private static final double MAX_DETECTION_CONFIDENCE = 0.95;
    private static final double MAX_DETECTION_AREA = 0.2;

    private static final ImmutableMap<String, ObjectDetectionClass> SIMULATION_DETECTION_CLASS_MAP =
            ImmutableMap.<String, ObjectDetectionClass>builder()
                    .put("Fuel", ObjectDetectionClass.FUEL)
                    .build();

    private static final FieldIntersection fieldIntersection = new FieldIntersection(
            FieldIntersectionOptions.DEFAULT.withRampCollider(false).withTransparentColliders(false));

    private static final int MAX_DETECTIONS_PER_FRAME = 127;

    /**
     * The next update time in seconds. Used for simulating realistic camera behavior.
     */
    private double nextUpdateTime = 0.0;

    private SortedMap<Double, List<ObjectDetectionResult>> frameQueue;

    private ImmutableSortedMap<ObjectDetectionClass, Integer> detectionClassReverseMap;

    /**
     * Constructs a MapleSimCamera with the given name and physical camera type and default settings.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera. Used for visibility checks and noise in realistic mode.
     * @param detectionClassMap The map of detection class IDs to ObjectDetectionClass enum.
     */
    public MapleSimCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d robotToCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        super(name, physicalCamera, robotToCamera, detectionClassMap);
        this.frameQueue = new TreeMap<>();
        this.detectionClassReverseMap = ContainerUtils.reverseMap(detectionClassMap);
    }

    /**
     * Checks if the camera is connected.
     * <p>In simulation, the camera is always connected.
     * @return True if the camera is connected, false otherwise.
     */
    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void recordMatchReplay() {
        // not supported
    }

    /**
     * Sets the detection class map.
     * @param detectionClassMap The detection class map.
     */
    @Override
    public void setDetectionClassMap(ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        super.setDetectionClassMap(detectionClassMap);
        this.detectionClassReverseMap = ContainerUtils.reverseMap(detectionClassMap);
    }

    /**
     * Makes object detections from the camera.
     * @return A list of object detection results.
     */
    @Override
    protected List<ObjectDetectionResult> makeDetections() {
        double currentTime = Timer.getTimestamp();

        if (currentTime >= nextUpdateTime) {
            performCapture();
        }

        return aggregateFrameQueue(currentTime);
    }

    private void performCapture() {
        PhysicalCamera camera = getPhysicalCamera();
        nextUpdateTime = Timer.getTimestamp() + 1.0 / camera.fps;
        double latency = Milliseconds.of(SimpleMath.gaussianNoise(camera.latencyMs, camera.latencyStdDevMs)
                        + SimpleMath.gaussianNoise(NETWORK_LATENCY_MS, NETWORK_LATENCY_STD_DEV_MS))
                .in(Seconds);

        List<ObjectDetectionResult> results = getDetections();
        frameQueue.put(Timer.getTimestamp() + latency, results);
    }

    private List<ObjectDetectionResult> getDetections() {
        Pose3d fieldToRobot = RobotContainer.model.getRobot();
        Pose3d fieldToCamera = fieldToRobot.transformBy(getRobotToCamera());

        return SimulatedArena.getInstance().gamePiecesOnField().stream()
                .map(target -> {
                    Pose3d fieldToTarget = target.getPose3d();
                    Pose3d cameraToTarget = fieldToTarget.relativeTo(fieldToCamera);

                    if (!getPhysicalCamera().isPointInFOV(cameraToTarget.getTranslation())
                            || fieldIntersection.collidesWithField(
                                    fieldToCamera.getTranslation().toTranslation2d(),
                                    fieldToTarget.getTranslation().toTranslation2d())) {
                        return Optional.empty();
                    }

                    ObjectDetectionClass detectionClass = SIMULATION_DETECTION_CLASS_MAP.get(target.getType());
                    if (detectionClass == null || !getDetectionClassMap().containsValue(detectionClass)) {
                        return Optional.empty();
                    }

                    double yawDegrees =
                            Units.radiansToDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));

                    double pitchDegrees = Units.radiansToDegrees(Math.atan(cameraToTarget.getZ()
                            / Math.sqrt(cameraToTarget.getX() * cameraToTarget.getX()
                                    + cameraToTarget.getY() * cameraToTarget.getY())));

                    return Optional.of(new ObjectDetectionResult(
                            detectionClassReverseMap.get(detectionClass),
                            MAX_DETECTION_CONFIDENCE,
                            yawDegrees,
                            pitchDegrees,
                            MAX_DETECTION_AREA,
                            Timer.getTimestamp(),
                            new ObjectDetectionResult.TargetCorner(0, 0),
                            new ObjectDetectionResult.TargetCorner(1, 0),
                            new ObjectDetectionResult.TargetCorner(1, 1),
                            new ObjectDetectionResult.TargetCorner(0, 1)));
                })
                .filter(Optional::isPresent)
                .map(Optional::get)
                .map(v -> (ObjectDetectionResult) v)
                .sorted((a, b) -> {
                    double distanceToTargetA = PhotonUtils.calculateDistanceToTargetMeters(
                            fieldToCamera.getZ(),
                            0, // target height doesn't matter for sorting by distance
                            fieldToCamera.getRotation().getY(),
                            Units.degreesToRadians(a.pitchDegrees()));
                    double distanceToTargetB = PhotonUtils.calculateDistanceToTargetMeters(
                            fieldToCamera.getZ(),
                            0, // target height doesn't matter for sorting by distance
                            fieldToCamera.getRotation().getY(),
                            Units.degreesToRadians(b.pitchDegrees()));
                    return Double.compare(distanceToTargetA, distanceToTargetB);
                })
                .limit(MAX_DETECTIONS_PER_FRAME)
                .toList();
    }

    private List<ObjectDetectionResult> aggregateFrameQueue(double thresholdTimestamp) {
        List<ObjectDetectionResult> results = new ArrayList<>();

        Iterator<Map.Entry<Double, List<ObjectDetectionResult>>> it =
                frameQueue.entrySet().iterator();

        while (it.hasNext()) {
            Map.Entry<Double, List<ObjectDetectionResult>> entry = it.next();
            if (entry.getKey() <= thresholdTimestamp) {
                List<ObjectDetectionResult> list = entry.getValue();
                if (list != null && !list.isEmpty()) {
                    results.addAll(list);
                }
                it.remove();
            } else {
                break;
            }
        }

        return results;
    }
}
