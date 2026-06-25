package frc.robot.utils.camera.positioned.poseestimation.io;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import frc.robot.utils.camera.positioned.poseestimation.PoseEstimationCamera;
import frc.robot.utils.libraries.LimelightHelpers;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import java.util.List;
import java.util.Optional;

/**
 * Limelight protocol camera for pose estimation.
 */
public class LimelightCamera extends PoseEstimationCamera {

    private static final double TIMEOUT_FPS = 3.0;
    private static final double TIMEOUT_SECONDS = 1.0 / TIMEOUT_FPS;

    private static final int THROTTLE_VALUE = 150;

    private double lastHeartbeatIncrementTime = 0.0;
    private double lastHeartbeatValue = -1.0;

    private LimelightIMUMode imuMode = LimelightIMUMode.EXTERNAL_SEED;

    private int throttle = 0;

    private int[] filter = Constants.Game.ALL_TAGS;

    public enum LimelightIMUMode {
        /**
         * No internal IMU processing. MT2 uses interpolated yaw from robot's gyro sent via SetRobotOrientation().
         */
        EXTERNAL_ONLY(0),
        /**
         * Internal IMU offset is calibrated to match external yaw each frame (seeding). MT2 still uses external yaw for botpose.
         */
        EXTERNAL_SEED(1),
        /**
         * 	Uses internal IMU's fused yaw only. No external input required.
         */
        INTERNAL_ONLY(2),
        /**
         * Complementary filter fuses internal IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects internal IMU drift.
         */
        INTERNAL_MT1_ASSIST(3),
        /**
         * Complementary filter fuses internal IMU with external yaw from SetRobotOrientation(). This is the recommended mode, as the internal IMU's 1khz update rate is utilized for frame-by-frame motion while the robot's IMU corrects for any drift over time.
         */
        INTERNAL_EXTERNAL_ASSIST(4);

        private final int num;

        private LimelightIMUMode(int num) {
            this.num = num;
        }
    }

    /**
     * Constructs a LimelightCamera with the given name and physical camera type and default settings.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     */
    public LimelightCamera(String name, PhysicalCamera physicalCamera, Transform3d toCamera) {
        super(name, physicalCamera, toCamera);
    }

    /**
     * Constructs a LimelightCamera with the given name and physical camera type.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param maxDistanceToCurrentEstimate Maximum distance from last pose to accept vision measurements.
     * @param unconstrainedMaxDistance Maximum distance to tag to use unconstrained measurements.
     * @param txtyMaxDistance Maximum distance to TXTY tag to use TXTY measurements.
     * @param rotationStdDevMultiplier Multiplier for rotation standard deviation when using rotation measurements.
     */
    protected LimelightCamera(
            String name,
            PhysicalCamera physicalCamera,
            double maxDistanceToCurrentEstimate,
            double unconstrainedMaxDistance,
            double txtyMaxDistance,
            double rotationStdDevMultiplier,
            Transform3d toCamera) {
        super(
                name,
                physicalCamera,
                maxDistanceToCurrentEstimate,
                unconstrainedMaxDistance,
                txtyMaxDistance,
                rotationStdDevMultiplier,
                toCamera);
    }

    private static Transform3d convertRobotToCamera(Transform3d robotToCamera) {
        return new Transform3d(
                robotToCamera.getX(),
                -robotToCamera.getY(),
                robotToCamera.getZ(),
                new Rotation3d(
                        robotToCamera.getRotation().getX(),
                        robotToCamera.getRotation().getY(),
                        robotToCamera.getRotation().getZ()));
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

    public LimelightIMUMode getLimelightIMUMode() {
        return imuMode;
    }

    public void setLimelightIMUMode(LimelightIMUMode mode) {
        imuMode = mode;
    }

    public int getThrottle() {
        return throttle;
    }

    public void setThrottle(int framesToSkip) {
        throttle = framesToSkip;
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
     * Sets the current pipeline index.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(getName(), pipeline);
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    @Override
    public List<CameraPoseEstimate> makeEstimates() {

        if (DriverStation.isEnabled()) {
            setLimelightIMUMode(LimelightIMUMode.INTERNAL_MT1_ASSIST);
        } else {
            setLimelightIMUMode(LimelightIMUMode.EXTERNAL_SEED);
        }

        if (DriverStation.isEnabled() || isForcingUnthrottled()) {
            setThrottle(0);
        } else {
            setThrottle(THROTTLE_VALUE);
        }

        updateCameraOrientation();

        Measurements measurements = getMeasurements();

        if (!isConnected() || measurements.mt1() == null || measurements.mt1().tagCount == 0) {
            return List.of();
        }

        CameraPoseEstimate estimate = new CameraPoseEstimate(
                getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                        ? measurements.mt13d()
                        : calculateRobotPose(measurements.mt13d(), measurements.mt1().timestampSeconds),
                Optional.ofNullable(measurements.mt2())
                        .map(m -> getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                                ? m.pose
                                : calculateRobotPose(new Pose3d(m.pose), m.timestampSeconds)
                                        .toPose2d()),
                measurements.txty(),
                measurements.mt1().timestampSeconds,
                measurements.mt1().latency,
                measurements.mt1().tagCount,
                measurements.mt1().avgTagDist,
                measurements.mt1().avgTagArea);

        return List.of(estimate);
    }

    /**
     * Updates the camera orientation in the Limelight network tables.
     */
    private void updateCameraOrientation() {
        Rotation2d estimatedRotation =
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation();

        double rollRate = RobotContainer.poseSensorFusion.imu.getRollRate().in(DegreesPerSecond);
        double pitchRate = RobotContainer.poseSensorFusion.imu.getPitchRate().in(DegreesPerSecond);
        double yawRate = RobotContainer.poseSensorFusion.imu.getYawRate().in(DegreesPerSecond);

        Transform3d robotToCamera = convertRobotToCamera(
                getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                        ? getRobotToCamera()
                        : getMechanismToCamera());

        LimelightHelpers.setCameraPose_RobotSpace(
                getName(),
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Units.radiansToDegrees(robotToCamera.getRotation().getX()),
                Units.radiansToDegrees(robotToCamera.getRotation().getY()),
                Units.radiansToDegrees(robotToCamera.getRotation().getZ()));

        Rotation3d orientation = new Rotation3d(
                        RobotContainer.poseSensorFusion.imu.getRoll().getRadians(),
                        RobotContainer.poseSensorFusion.imu.getPitch().getRadians(),
                        estimatedRotation.getRadians())
                .plus(getLastRobotToMechanism().getRotation());

        LimelightHelpers.SetIMUMode(getName(), imuMode.num);

        LimelightHelpers.SetThrottle(getName(), throttle);

        LimelightHelpers.SetFiducialIDFiltersOverride(getName(), filter);

        LimelightHelpers.SetRobotOrientation(
                getName(),
                Units.radiansToDegrees(orientation.getZ()),
                useVelocity() ? yawRate : 0,
                use3DRotation() ? Units.radiansToDegrees(orientation.getY()) : 0,
                useVelocity() ? pitchRate : 0,
                use3DRotation() ? Units.radiansToDegrees(orientation.getX()) : 0,
                useVelocity() ? rollRate : 0);
    }

    @Override
    public void setFilter(int[] filter) {
        this.filter = filter;
    }

    /**
     * Gets the measurements from the Limelight.
     * @return The measurements from the Limelight.
     */
    private Measurements getMeasurements() {
        Pose3d pose3d = LimelightHelpers.getBotPose3d_wpiBlue(getName());
        PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(getName());
        PoseEstimate measurementM2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
        List<TXTYMeasurement> txtyMeasurement = measurement == null
                ? List.of()
                : pnpDistanceTrigSolveStrategy(measurement.timestampSeconds, measurement.tagCount);

        return new Measurements(measurement, pose3d, measurementM2, txtyMeasurement);
    }

    /**
     * From PhotonVision
     * @param timestamp the timestamp of the measurement
     * @param tagCount number of tags seen
     * @return the TXTYMeasurements
     */
    @SuppressWarnings("java:S1941") // library code
    private List<TXTYMeasurement> pnpDistanceTrigSolveStrategy(double timestamp, int tagCount) {
        if (tagCount == 0) {
            return List.of();
        }

        Transform3d robotToCamera = getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                ? getRobotToCameraAt(timestamp).orElse(getRobotToCamera())
                : getMechanismToCamera();

        Optional<Pose2d> headingSampleOpt = RobotContainer.poseSensorFusion.getEstimatedPositionAt(timestamp);
        if (headingSampleOpt.isEmpty()) {
            return List.of();
        }
        Rotation2d headingSample = headingSampleOpt
                .get()
                .getRotation()
                .plus(sampleRobotToMechanism(timestamp)
                        .orElse(getLastRobotToMechanism())
                        .getRotation()
                        .toRotation2d());

        Pose3d bestCameraToTarget = LimelightHelpers.toPose3D(LimelightHelpers.getTargetPose_CameraSpace(getName()));

        Translation2d camToTagTranslation = new Translation3d(
                        bestCameraToTarget.getTranslation().getNorm(),
                        new Rotation3d(
                                0,
                                -Math.toRadians(LimelightHelpers.getTYNC(getName())),
                                -Math.toRadians(LimelightHelpers.getTXNC(getName()))))
                .rotateBy(robotToCamera.getRotation())
                .toTranslation2d()
                .rotateBy(headingSample);

        int fiducialId = (int) LimelightHelpers.getFiducialID(getName());

        Optional<Pose3d> tagPoseOpt = Constants.Game.APRILTAG_LAYOUT.getTagPose(fiducialId);
        if (tagPoseOpt.isEmpty()) {
            return List.of();
        }
        Pose2d tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation = tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
                robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(headingSample);

        Pose2d robotPose = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);

        return List.of(new TXTYMeasurement(
                getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                        ? robotPose
                        : calculateRobotPose(new Pose3d(robotPose), timestamp).toPose2d(),
                bestCameraToTarget.getTranslation().getNorm(),
                fiducialId));
    }

    /**
     * A record representing the measurements from the Limelight.
     * @param mt1 The first pose estimate measurement.
     * @param mt13d The first 3D pose estimate measurement.
     * @param mt2 The second pose estimate measurement.
     * @param txty The list of TXTY measurements.
     */
    private record Measurements(PoseEstimate mt1, Pose3d mt13d, PoseEstimate mt2, List<TXTYMeasurement> txty) {}
}
