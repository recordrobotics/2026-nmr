package frc.robot.utils.camera.positioned.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.Optional;

/**
 * A record representing a camera pose estimate.
 * @param unconstrainedPose The unconstrained pose estimate.
 * @param constrainedPose The constrained pose estimate, if available.
 * @param txtyMeasurements The list of TXTY measurements available from the estimate.
 * @param timestampSeconds The timestamp of the estimate in seconds.
 * @param latency The latency of the estimate in seconds.
 * @param tagCount The number of tags detected in the estimate.
 * @param avgTagDist The average distance to the detected tags.
 * @param avgTagArea The average area of the detected tags.
 */
public record CameraPoseEstimate(
        Pose3d unconstrainedPose,
        Optional<Pose2d> constrainedPose,
        List<TXTYMeasurement> txtyMeasurements,
        double timestampSeconds,
        double latency,
        int tagCount,
        double avgTagDist,
        double avgTagArea) {

    /**
     * A record representing a TXTY measurement.
     * @param pose The pose associated with the TXTY measurement.
     * @param distToCamera The distance to the camera for the TXTY measurement.
     * @param tagId The ID of the tag associated with the TXTY measurement.
     */
    public record TXTYMeasurement(Pose2d pose, double distToCamera, int tagId) {}
}
