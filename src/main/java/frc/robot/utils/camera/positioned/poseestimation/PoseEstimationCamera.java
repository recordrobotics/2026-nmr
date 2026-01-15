package frc.robot.utils.camera.positioned.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.PoseSensorFusion;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.PositionedCamera;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Abstract class for cameras that provide pose estimates.
 * <p>Supports both constrained and unconstrained pose estimates as well as a way to force using unconstrained estimates.
 * <p>Allows rotation measurements to be optionally trusted or ignored.
 */
public abstract class PoseEstimationCamera extends PositionedCamera<PoseEstimationCamera> {

    public static final double DEFAULT_MAX_DISTANCE_TO_CURRENT_ESTIMATE = 10.0;
    public static final double DEFAULT_UNCONSTRAINED_MAX_DISTANCE = Units.feetToMeters(7);
    public static final double DEFAULT_TXTY_MAX_DISTANCE = 1.0;
    public static final double DEFAULT_ROTATION_STDDEV_MULTIPLIER = 5.0;

    /**
     * The dashboard entry for whether to trust rotation measurements.
     */
    private final LoggedNetworkBoolean useRotationToggle;
    /**
     * The dashboard entry for forcing unconstrained measurements.
     * <p>If true, unconstrained measurements will always be used regardless of distance.
     * <p>Useful for setting initial rotation on the field.
     */
    private final LoggedNetworkBoolean forceUnconstrainedToggle;

    private final LoggedNetworkBoolean forceUnconstrainedWhenDisabledToggle;

    /**
     * The dashboard entry for forcing unthrottled measurements.
     */
    private final LoggedNetworkBoolean forceUnthrottledToggle;

    /**
     * List of estimates from the camera from last periodic.
     */
    private List<CameraPoseEstimate> cachedEstimates = List.of();

    /**
     * Last used standard deviation for measurements.
     * Used for logging.
     */
    private double lastStdDev = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;

    private boolean ignored = false;

    /**
     * Whether to trust rotation measurements.
     */
    private boolean useRotation = true;

    /**
     * Whether to force using unconstrained measurements.
     */
    private boolean forceUnconstrained = false;

    private boolean forceUnconstrainedWhenDisabled = false;

    /**
     * Whether to force unthrottled detections
     */
    private boolean forceUnthrottled = false;

    /**
     * Whether to compute robot-to-camera transforms for each estimate based on a known robot pose.
     */
    private boolean computeRobotToCamera = false;

    /**
     * A known robot pose to use for calculating robot-to-camera transforms for each estimate.
     */
    private Pose3d knownRobotPose = Pose3d.kZero;

    /**
     * Maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     */
    private double maxDistanceToCurrentEstimate;

    /**
     * Maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     */
    private double unconstrainedMaxDistance;

    /**
     * Maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     */
    private double txtyMaxDistance;

    /**
     * Multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     */
    private double rotationStdDevMultiplier;

    private LinearFilter rtcFilterX = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterY = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterZ = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterQX = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterQY = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterQZ = LinearFilter.movingAverage(10);
    private LinearFilter rtcFilterQW = LinearFilter.movingAverage(10);

    private Transform3d lastRTC = Transform3d.kZero;
    private double lastRTCTime = Timer.getTimestamp();

    /**
     * Constructs a PoseEstimationCamera with the given name and physical camera type and default settings.
     * <p>Sets up dashboard entries for configuring rotation trust and forcing unconstrained measurements.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     */
    protected PoseEstimationCamera(String name, PhysicalCamera physicalCamera, Transform3d toCamera) {
        this(
                name,
                physicalCamera,
                DEFAULT_MAX_DISTANCE_TO_CURRENT_ESTIMATE,
                DEFAULT_UNCONSTRAINED_MAX_DISTANCE,
                DEFAULT_TXTY_MAX_DISTANCE,
                DEFAULT_ROTATION_STDDEV_MULTIPLIER,
                toCamera);
    }

    /**
     * Constructs a PoseEstimationCamera with the given name and physical camera type.
     * <p>Sets up dashboard entries for configuring rotation trust and forcing unconstrained measurements.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param maxDistanceToCurrentEstimate Maximum distance from last pose to accept vision measurements.
     * @param unconstrainedMaxDistance Maximum distance to tag to use unconstrained measurements.
     * @param txtyMaxDistance Maximum distance to TXTY tag to use TXTY measurements.
     * @param rotationStdDevMultiplier Multiplier for rotation standard deviation when using rotation measurements.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     */
    protected PoseEstimationCamera(
            String name,
            PhysicalCamera physicalCamera,
            double maxDistanceToCurrentEstimate,
            double unconstrainedMaxDistance,
            double txtyMaxDistance,
            double rotationStdDevMultiplier,
            Transform3d toCamera) {
        super(name, physicalCamera, toCamera);

        this.maxDistanceToCurrentEstimate = maxDistanceToCurrentEstimate;
        this.unconstrainedMaxDistance = unconstrainedMaxDistance;
        this.txtyMaxDistance = txtyMaxDistance;
        this.rotationStdDevMultiplier = rotationStdDevMultiplier;

        // Allow configuration from dashboard
        useRotationToggle = new LoggedNetworkBoolean(getPrefix() + "UseRotation", useRotation);
        forceUnconstrainedToggle = new LoggedNetworkBoolean(getPrefix() + "ForceUnconstrained", forceUnconstrained);
        forceUnthrottledToggle = new LoggedNetworkBoolean(getPrefix() + "ForceUnthrottled", forceUnthrottled);
        forceUnconstrainedWhenDisabledToggle = new LoggedNetworkBoolean(
                getPrefix() + "ForceUnconstrainedWhenDisabled", forceUnconstrainedWhenDisabled);
    }

    /**
     * Whether rotation measurements are being trusted.
     * @return True if rotation measurements are trusted, false otherwise.
     */
    public boolean isUsingRotation() {
        return useRotation;
    }

    public boolean isIgnored() {
        return ignored;
    }

    public PoseEstimationCamera setIgnore(boolean ignore) {
        ignored = ignore;
        return this;
    }

    /**
     * Whether unconstrained measurements are being forced.
     * @return True if unconstrained measurements are forced, false otherwise.
     */
    public boolean isForcingUnconstrained() {
        return forceUnconstrained;
    }

    public boolean isForcingUnthrottled() {
        return forceUnthrottled;
    }

    public boolean isForcingUnconstrainedWhenDisabled() {
        return forceUnconstrainedWhenDisabled;
    }

    /**
     * Sets whether to trust rotation measurements.
     * @param useRotation True to trust rotation measurements, false to ignore them.
     */
    public PoseEstimationCamera setUseRotation(boolean useRotation) {
        this.useRotation = useRotation;
        useRotationToggle.set(useRotation);
        return this;
    }

    /**
     * Sets whether to force using unconstrained measurements.
     * @param forceUnconstrained True to force using unconstrained measurements, false to use normal logic.
     */
    public PoseEstimationCamera setForceUnconstrained(boolean forceUnconstrained) {
        this.forceUnconstrained = forceUnconstrained;
        forceUnconstrainedToggle.set(forceUnconstrained);
        return this;
    }

    public PoseEstimationCamera setForceUnthrottled(boolean forceUnthrottled) {
        this.forceUnthrottled = forceUnthrottled;
        forceUnthrottledToggle.set(forceUnthrottled);
        return this;
    }

    public PoseEstimationCamera setForceUnconstrainedWhenDisabled(boolean forceUnconstrainedWhenDisabled) {
        this.forceUnconstrainedWhenDisabled = forceUnconstrainedWhenDisabled;
        forceUnconstrainedWhenDisabledToggle.set(forceUnconstrainedWhenDisabled);
        return this;
    }

    public PoseEstimationCamera setComputeRobotToCamera(boolean computeRobotToCamera, Pose3d knownRobotPose) {
        this.computeRobotToCamera = computeRobotToCamera;
        this.knownRobotPose = knownRobotPose;
        return this;
    }

    public PoseEstimationCamera setComputeRobotToCamera(boolean computeRobotToCamera) {
        this.computeRobotToCamera = computeRobotToCamera;
        return this;
    }

    public boolean isComputingRobotToCamera() {
        return computeRobotToCamera;
    }

    public Pose3d getKnownRobotPose() {
        return knownRobotPose;
    }

    /**
     * Whether the camera has valid vision estimates.
     * @return True if there are valid estimates, false otherwise.
     */
    public boolean hasVision() {
        return !cachedEstimates.isEmpty();
    }

    /**
     * Gets the number of tags detected in the latest estimate.
     * @return The number of tags detected, or 0 if no vision.
     */
    public int getNumTags() {
        if (!hasVision()) return 0;
        return cachedEstimates.get(cachedEstimates.size() - 1).tagCount();
    }

    /**
     * Gets the last standard deviation used in vision measurements.
     * @return The last standard deviation.
     */
    public double getLastStdDev() {
        return lastStdDev;
    }

    /**
     * Gets the maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     * @return The maximum distance to accept vision measurements.
     */
    public double getMaxDistanceToCurrentEstimate() {
        return maxDistanceToCurrentEstimate;
    }

    /**
     * Gets the maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     * @return The maximum distance to tag for unconstrained measurements.
     */
    public double getUnconstrainedMaxDistance() {
        return unconstrainedMaxDistance;
    }

    /**
     * Gets the maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     * @return The maximum distance to TXTY tag for TXTY measurements.
     */
    public double getTxtyMaxDistance() {
        return txtyMaxDistance;
    }

    /**
     * Gets the multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     * @return The rotation standard deviation multiplier.
     */
    public double getRotationStdDevMultiplier() {
        return rotationStdDevMultiplier;
    }

    /**
     * Sets the maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     * @param maxDistanceToCurrentEstimate The maximum distance to accept vision measurements.
     */
    public PoseEstimationCamera setMaxDistanceToCurrentEstimate(double maxDistanceToCurrentEstimate) {
        this.maxDistanceToCurrentEstimate = maxDistanceToCurrentEstimate;
        return this;
    }

    /**
     * Sets the maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     * @param unconstrainedMaxDistance The maximum distance to tag for unconstrained measurements.
     */
    public PoseEstimationCamera setUnconstrainedMaxDistance(double unconstrainedMaxDistance) {
        this.unconstrainedMaxDistance = unconstrainedMaxDistance;
        return this;
    }

    /**
     * Sets the maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     * @param txtyMaxDistance The maximum distance to TXTY tag for TXTY measurements.
     */
    public PoseEstimationCamera setTxtyMaxDistance(double txtyMaxDistance) {
        this.txtyMaxDistance = txtyMaxDistance;
        return this;
    }

    /**
     * Sets the multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     * @param rotationStdDevMultiplier The rotation standard deviation multiplier.
     */
    public PoseEstimationCamera setRotationStdDevMultiplier(double rotationStdDevMultiplier) {
        this.rotationStdDevMultiplier = rotationStdDevMultiplier;
        return this;
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    protected abstract List<CameraPoseEstimate> makeEstimates();

    public abstract void setFilter(int[] filter);

    /**
     * Periodically updates the cached pose estimates from the camera.
     * <p>This method should be called regularly (e.g. in a main periodic loop).
     */
    @Override
    public void periodic() {
        if (!isEnabled()) {
            cachedEstimates = List.of();
            return;
        }

        cachedEstimates = makeEstimates();
    }

    /***
     * Adds vision measurements to the sensor fusion.
     * @param fusion The pose sensor fusion to add measurements to.
     * @param currentEstimate The current estimated pose from the fusion.
     * @param txtyId The ID of the TXTY measurement target.
     */
    public void addVisionMeasurements(PoseSensorFusion fusion, Pose2d currentEstimate, int txtyId) {
        for (CameraPoseEstimate estimate : cachedEstimates) {

            if (computeRobotToCamera) {
                lastRTC = getRobotToCamera(knownRobotPose, estimate);
                lastRTCTime = Timer.getTimestamp();
            }

            double stdDev = calculateStdDevs(fusion, estimate);

            Pose2d pose;

            if (forceUnconstrained || (forceUnconstrainedWhenDisabled && DriverStation.isDisabled())) {
                pose = estimate.unconstrainedPose().toPose2d();
            } else {
                Optional<TXTYMeasurement> txtyMeasurement = findValidTXTY(estimate, txtyId);

                if (txtyMeasurement.isPresent()) {
                    pose = txtyMeasurement.get().pose();
                    stdDev = getPhysicalCamera().txtyStdDevs;
                } else {
                    pose = choosePoseEstimate(estimate);
                }
            }

            if (SimpleMath.isInField(pose)
                    && (pose.getTranslation().getDistance(currentEstimate.getTranslation())
                                    <= maxDistanceToCurrentEstimate
                            || !SimpleMath.isInField(currentEstimate.getTranslation()))) {
                lastStdDev = stdDev;
                fusion.addVisionMeasurement(
                        pose,
                        estimate.timestampSeconds(),
                        VecBuilder.fill(
                                stdDev,
                                stdDev,
                                useRotation
                                        ? stdDev * rotationStdDevMultiplier
                                        : PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS));
            }
        }
    }

    public void applyCalculatedRobotToCamera() {
        if (Timer.getTimestamp() - lastRTCTime < 2.0) {
            setMechanismToCamera(lastRTC);
        }
    }

    /**
     * Calculates the standard deviations for a given estimate.
     * @param fusion The pose sensor fusion.
     * @param estimate The camera pose estimate.
     * @return The calculated standard deviations.
     */
    private double calculateStdDevs(PoseSensorFusion fusion, CameraPoseEstimate estimate) {
        double stdDev = getPhysicalCamera().calculateStdDevs(estimate.avgTagDist());

        Optional<Pose2d> closestPose = fusion.getEstimatedPositionAt(estimate.timestampSeconds());
        if (closestPose.isPresent() && SimpleMath.isInField(closestPose.get())) {
            double distanceToClosest = estimate.unconstrainedPose()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(closestPose.get().getTranslation());
            stdDev += distanceToClosest / 2.0;
        }

        return stdDev;
    }

    /**
     * Finds a valid TXTY measurement within the maximum distance.
     * @param estimate The camera pose estimate.
     * @param txtyId The ID of the TXTY measurement target.
     * @return An optional TXTY measurement if found and valid.
     */
    private Optional<TXTYMeasurement> findValidTXTY(CameraPoseEstimate estimate, int txtyId) {
        for (TXTYMeasurement m : estimate.txtyMeasurements()) {
            if (m.tagId() == txtyId && m.distToCamera() <= txtyMaxDistance) {
                return Optional.of(m);
            }
        }

        return Optional.empty();
    }

    /**
     * Chooses between unconstrained and constrained pose estimates based on average tag distance.
     * @param estimate The camera pose estimate.
     * @return An optional chosen pose estimate.
     */
    private Pose2d choosePoseEstimate(CameraPoseEstimate estimate) {
        if (estimate.avgTagDist() <= unconstrainedMaxDistance) {
            return estimate.unconstrainedPose().toPose2d();
        } else {
            return estimate.constrainedPose()
                    .orElse(estimate.unconstrainedPose().toPose2d());
        }
    }

    /**
     * Calculates the robot-to-camera transform for a given estimate based on a known robot pose.
     * @param robotPose The known robot pose.
     * @param estimate The camera pose estimate.
     * @return The robot-to-camera transform.
     */
    private Transform3d getRobotToCamera(Pose3d robotPose, CameraPoseEstimate estimate) {
        Transform3d currentRobotToCamera = getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                ? getRobotToCameraAt(estimate.timestampSeconds()).orElse(getRobotToCamera())
                : getMechanismToCamera();
        Pose3d cameraPose = estimate.unconstrainedPose().transformBy(currentRobotToCamera);
        Logger.recordOutput(getPrefix() + "CM", cameraPose);

        Transform3d rtc = cameraPose.minus(robotPose);

        double filteredX = rtcFilterX.calculate(rtc.getX());
        double filteredY = rtcFilterY.calculate(rtc.getY());
        double filteredZ = rtcFilterZ.calculate(rtc.getZ());
        Quaternion q = rtc.getRotation().getQuaternion();
        double filteredQX = rtcFilterQX.calculate(q.getX());
        double filteredQY = rtcFilterQY.calculate(q.getY());
        double filteredQZ = rtcFilterQZ.calculate(q.getZ());
        double filteredQW = rtcFilterQW.calculate(q.getW());

        Transform3d filteredRtc = new Transform3d(
                filteredX,
                filteredY,
                filteredZ,
                new Rotation3d(new Quaternion(filteredQW, filteredQX, filteredQY, filteredQZ)));

        Logger.recordOutput(getPrefix() + "RTC", filteredRtc);
        return filteredRtc;
    }

    /**
     * Logs camera values.
     * <ul>
     * <li>[UnconstrainedPose]: Unconstrained Pose</li>
     * <li>[ConstrainedPose]: Constrained Pose</li>
     * <li>[AvgDist]: Average Tag Distance</li>
     * <li>[NumTags]: Number of Tags</li>
     * <li>[StdDev]: Standard Deviation</li>
     * <li>[TXTY]: TXTY Measurements</li>
     * <li>[HasVision]: Has Vision</li>
     * </ul>
     */
    @Override
    public void logValues() {
        super.logValues();

        String prefix = getPrefix();

        if (!cachedEstimates.isEmpty()) {
            CameraPoseEstimate lastEstimate = cachedEstimates.get(cachedEstimates.size() - 1);
            Logger.recordOutput(prefix + "UnconstrainedPose", lastEstimate.unconstrainedPose());
            lastEstimate.constrainedPose().ifPresent(p -> Logger.recordOutput(prefix + "ConstrainedPose", p));
            Logger.recordOutput(prefix + "AvgDist", lastEstimate.avgTagDist());
            Logger.recordOutput(prefix + "NumTags", lastEstimate.tagCount());
            Logger.recordOutput(prefix + "StdDev", lastStdDev);
            Logger.recordOutput(
                    prefix + "TXTY",
                    lastEstimate.txtyMeasurements().stream().map(v -> v.pose()).toArray(Pose2d[]::new));
        }

        Logger.recordOutput(prefix + "HasVision", hasVision());
        Logger.recordOutput(prefix + "ToCamera", getMechanismToCamera());
        Logger.recordOutput(prefix + "Ignored", isIgnored());

        // Update from dashboard
        useRotation = useRotationToggle.get();
        forceUnconstrained = forceUnconstrainedToggle.get();
        forceUnconstrainedWhenDisabled = forceUnconstrainedWhenDisabledToggle.get();
        forceUnthrottled = forceUnthrottledToggle.get();
    }
}
