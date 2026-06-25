package frc.robot.utils.camera.positioned.objectdetection.io;

import static edu.wpi.first.units.Units.Seconds;

import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionClass;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionResult;
import frc.robot.utils.libraries.LimelightHelpers;
import java.util.Arrays;
import java.util.List;

public class LimelightCamera extends ObjectDetectionCamera {

    /**
     * Timeout parameters for determining if the camera is connected.
     */
    private static final double TIMEOUT_FPS = 3.0;

    private static final double TIMEOUT_SECONDS = 1.0 / TIMEOUT_FPS;

    /**
     * Default confidence value for detections since Limelight does not provide confidence.
     */
    private static final double DEFAULT_CONFIDENCE = 0.85;

    /**
     * The time and value when the heartbeat last incremented.
     */
    private double lastHeartbeatIncrementTime = 0.0;

    private double lastHeartbeatValue = -1.0;

    /**
     * Constructs a LimelightCamera with the given name and physical camera type and default settings.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera.
     * @param detectionClassMap The map of detection class IDs to ObjectDetectionClass enum.
     */
    public LimelightCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d robotToCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        super(name, physicalCamera, robotToCamera, detectionClassMap);
    }

    /**
     * Checks if the camera is connected.
     * <p>This is different from being enabled. A camera can be enabled but not connected.
     * @return True if the camera is connected, false otherwise.
     */
    @Override
    public boolean isConnected() {
        double heartbeat = LimelightHelpers.getHeartbeat(getName());
        double currentTime = Timer.getTimestamp();

        if (heartbeat > lastHeartbeatValue) {
            lastHeartbeatIncrementTime = currentTime;
            lastHeartbeatValue = heartbeat;
            return true;
        } else {
            return currentTime - lastHeartbeatIncrementTime < TIMEOUT_SECONDS;
        }
    }

    /**
     * Sets the current pipeline index.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(getName(), pipeline);
    }

    public void configureRewind(boolean enabled) {
        LimelightHelpers.setRewindEnabled(getName(), enabled);
    }

    @Override
    public void recordMatchReplay() {
        recordReplay(Seconds.of(165));
    }

    /**
     * Triggers a rewind capture (only LL4)
     * @param duration The duration of the capture to record, up to 165 seconds.
     */
    public void recordReplay(Time duration) {
        LimelightHelpers.triggerRewindCapture(getName(), duration.in(Seconds));
    }

    /**
     * Makes object detections from the camera.
     * @return A list of object detection results.
     */
    @Override
    protected List<ObjectDetectionResult> makeDetections() {
        if (!isConnected()) {
            return List.of();
        }

        TimestampedDetection[] rawDetections = getRawDetectionsTimestamped(getName());

        return Arrays.stream(rawDetections)
                .map(rawDetection -> new ObjectDetectionResult(
                        rawDetection.classId,
                        DEFAULT_CONFIDENCE, // Limelight does not provide confidence, so we use a default value
                        rawDetection.txnc,
                        rawDetection.tync,
                        rawDetection.ta,
                        rawDetection.timestamp,
                        new ObjectDetectionResult.TargetCorner(rawDetection.corner0_X, rawDetection.corner0_Y),
                        new ObjectDetectionResult.TargetCorner(rawDetection.corner1_X, rawDetection.corner1_Y),
                        new ObjectDetectionResult.TargetCorner(rawDetection.corner2_X, rawDetection.corner2_Y),
                        new ObjectDetectionResult.TargetCorner(rawDetection.corner3_X, rawDetection.corner3_Y)))
                .toList();
    }

    /**
     * Gets the latest raw neural detector results from NetworkTables
     * <p>Modified from Limelight's official LimelightHelpers.getRawDetections method to include timestamps.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of TimestampedDetection objects containing detection details
     */
    @SuppressWarnings({"java:S109", "java:S1941", "java:S117"}) // library code
    private static TimestampedDetection[] getRawDetectionsTimestamped(String limelightName) {
        DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "rawdetections");

        TimestampedDoubleArray tsValue = entry.getAtomic();

        double[] rawDetectionArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        int valsPerEntry = 12;
        if (rawDetectionArray.length % valsPerEntry != 0) {
            return new TimestampedDetection[0];
        }

        int numDetections = rawDetectionArray.length / valsPerEntry;
        TimestampedDetection[] rawDetections = new TimestampedDetection[numDetections];

        double totalLatency = LimelightHelpers.getLatency_Capture(limelightName)
                + LimelightHelpers.getLatency_Pipeline(limelightName);

        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (totalLatency / 1000.0);

        for (int i = 0; i < numDetections; i++) {
            int baseIndex = i * valsPerEntry; // Starting index for this detection's data
            int classId = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            rawDetections[i] = new TimestampedDetection(
                    classId,
                    txnc,
                    tync,
                    ta,
                    corner0_X,
                    corner0_Y,
                    corner1_X,
                    corner1_Y,
                    corner2_X,
                    corner2_Y,
                    corner3_X,
                    corner3_Y,
                    adjustedTimestamp);
        }

        return rawDetections;
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    /**
     * Represents a Limelight Raw Neural Detector result from Limelight's NetworkTables output.
     */
    public record TimestampedDetection(
            int classId,
            double txnc,
            double tync,
            double ta,
            double corner0_X,
            double corner0_Y,
            double corner1_X,
            double corner1_Y,
            double corner2_X,
            double corner2_Y,
            double corner3_X,
            double corner3_Y,
            double timestamp) {}
}
