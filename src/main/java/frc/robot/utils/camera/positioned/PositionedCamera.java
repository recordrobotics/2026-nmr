package frc.robot.utils.camera.positioned;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.PhysicalCamera;
import java.util.Optional;

/**
 * Abstract camera class for cameras that have a known position on the robot, such as a camera mounted on a mount an arm or elevator.
 */
public abstract class PositionedCamera<T extends PositionedCamera<T>> extends GenericCamera {

    private static final double BUFFER_DURATION = 1.5;

    private boolean use3DRotation = true;
    private boolean useVelocity = true;
    private Transform3d mechanismToCamera = Transform3d.kZero;
    private DynamicPositionMode dynamicPositionMode = DynamicPositionMode.ROBOT_TO_CAMERA;

    private final TimeInterpolatableBuffer<Pose3d> robotToMechanismBuffer =
            TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
    private Transform3d lastRobotToMechanism = Transform3d.kZero;

    public enum DynamicPositionMode {
        /**
         * The camera calculates the robot's pose directly, using a robot-to-camera transform that is
         * updated every periodic.
         * This is easier to debug in the camera dashboards, however is
         * less accurate when the mechanism is in motion due to latency,
         * depending on the camera implementation.
         */
        ROBOT_TO_CAMERA,
        /**
         * The camera calculates the mechanism's pose, using a CONSTANT mechanism-to-camera transform.
         * Then every periodic a robot-to-mechanism transform is calculated using
         * the timetamp from the camera and combined with the mechanim pose.
         * This is much more accurate with moving mechanisms, but is harder to
         * check robot pose estimates in the camera dashboards since the robot pose is
         * not directly calculated by the camera.
         */
        MECHANISM_TO_CAMERA
    }

    /**
     * Constructs a PoseEstimationCamera with the given name and physical camera type and default settings.
     * <p>Sets up dashboard entries for configuring rotation trust and forcing unconstrained measurements.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     */
    protected PositionedCamera(String name, PhysicalCamera physicalCamera, Transform3d toCamera) {
        super(name, physicalCamera);
        this.mechanismToCamera = toCamera;
    }

    /**
     * Returns whether the camera should use 3D rotation (i.e. full rotation in 3D space) or just yaw rotation.
     * @return Whether the camera should use 3D rotation or just yaw rotation.
     */
    public boolean use3DRotation() {
        return use3DRotation;
    }

    /**
     * Sets whether the camera should use 3D rotation (i.e. full rotation in 3D space) or just yaw rotation.
     * @param use3DRotation Whether the camera should use 3D rotation or just yaw rotation.
     */
    public void setUse3DRotation(boolean use3DRotation) {
        this.use3DRotation = use3DRotation;
    }

    /**
     * Returns whether the camera should use angular velocity measurements from the nav sensor for orientation.
     * @return Whether the camera should use angular velocity measurements.
     */
    public boolean useVelocity() {
        return useVelocity;
    }

    /**
     * Sets whether the camera should use angular velocity measurements from the nav sensor for orientation.
     * @param useVelocity Whether the camera should use angular velocity measurements.
     */
    public void setUseVelocity(boolean useVelocity) {
        this.useVelocity = useVelocity;
    }

    /**
     * Returns the transform from the mechanism to the camera. If {@link #updateRobotToMechanism(Transform3d, double)} is never called, this will be used as the transform from the robot to the camera.
     * @return The transform from the mechanism to the camera.
     */
    public Transform3d getMechanismToCamera() {
        return mechanismToCamera;
    }

    /**
     * Sets the transform from the mechanism to the camera. If {@link #updateRobotToMechanism(Transform3d, double)} is never called, this will be used as the transform from the robot to the camera.
     * @param mechanismToCamera The transform from the mechanism to the camera.
     */
    public void setMechanismToCamera(Transform3d mechanismToCamera) {
        this.mechanismToCamera = mechanismToCamera;
    }

    /**
     * Returns the dynamic position mode of the camera, which determines how the camera calculates the robot's pose.
     * @return The dynamic position mode.
     */
    public DynamicPositionMode getDynamicPositionMode() {
        return dynamicPositionMode;
    }

    /**
     * Sets the dynamic position mode of the camera, which determines how the camera calculates the robot's pose.
     * @param mode The dynamic position mode to set.
     */
    @SuppressWarnings("unchecked")
    public T setDynamicPositionMode(DynamicPositionMode mode) {
        this.dynamicPositionMode = mode;
        return (T) this;
    }

    /**
     * Updates the robot-to-mechanism transform buffer with a new sample.
     * This should be called every periodic with the latest robot-to-mechanism transform and timestamp.
     *
     * NOTE: Don't call this method if robot-to-camera is constant!
     * The camera will use the mechanism-to-camera transform as the robot-to-camera transform if this method is never called.
     * @param robotToMechanism The robot-to-mechanism transform.
     * @param timestamp The timestamp of the sample.
     */
    public void updateRobotToMechanism(Transform3d robotToMechanism, double timestamp) {
        robotToMechanismBuffer.addSample(
                timestamp, new Pose3d(robotToMechanism.getTranslation(), robotToMechanism.getRotation()));
        lastRobotToMechanism = robotToMechanism;
    }

    public boolean hasRobotToMechanism() {
        return !robotToMechanismBuffer.getInternalBuffer().isEmpty();
    }

    /**
     * Return the robot-to-mechanism pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    private Optional<Pose3d> sampleAt(double timestampSeconds) {
        // If there are no updates to sample, skip.
        if (robotToMechanismBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Make sure timestamp matches the sample from the pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestTimestamp = robotToMechanismBuffer.getInternalBuffer().firstKey();
        double newestTimestamp = robotToMechanismBuffer.getInternalBuffer().lastKey();
        timestampSeconds = MathUtil.clamp(timestampSeconds, oldestTimestamp, newestTimestamp);
        return robotToMechanismBuffer.getSample(timestampSeconds);
    }

    /**
     * Returns the robot-to-mechanism transform at a given timestamp, if the buffer is not empty.
     * @param timestamp The timestamp of the sample.
     * @return The transform at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    public Optional<Transform3d> sampleRobotToMechanism(double timestamp) {
        Optional<Pose3d> robotToMechanismPose = sampleAt(timestamp);
        return robotToMechanismPose.map(pose -> new Transform3d(pose.getTranslation(), pose.getRotation()));
    }

    /**
     * Returns the last robot-to-mechanism transform that was added to the buffer. This is used as a fallback if the buffer is empty when sampling.
     * @return The last robot-to-mechanism transform that was added to the buffer.
     */
    public Transform3d getLastRobotToMechanism() {
        return lastRobotToMechanism;
    }

    /**
     * Returns the robot-to-camera transform. If {@link #updateRobotToMechanism(Transform3d, double)} is never called, this will return the mechanism-to-camera transform.
     * @return The robot-to-camera transform.
     */
    public final Transform3d getRobotToCamera() {
        return lastRobotToMechanism.plus(mechanismToCamera);
    }

    /**
     * Returns the robot-to-camera transform at a given timestamp. If {@link #updateRobotToMechanism(Transform3d, double)} is never called, this will return the mechanism-to-camera transform.
     * @param timestamp The timestamp of the sample.
     * @return The robot-to-camera transform at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    public Optional<Transform3d> getRobotToCameraAt(double timestamp) {
        Optional<Transform3d> robotToMechanism = sampleRobotToMechanism(timestamp);
        return robotToMechanism.map(rm -> rm.plus(mechanismToCamera));
    }

    /**
     * Calculates the robot pose from the mechanism pose and the robot-to-camera transform at the given timestamp.
     * @param mechanismPose The mechanism pose.
     * @param timestamp The timestamp of the sample.
     * @return The robot pose at the given timestamp.
     */
    protected Pose3d calculateRobotPose(Pose3d mechanismPose, double timestamp) {
        return sampleRobotToMechanism(timestamp)
                .map(robotToMechanism -> mechanismPose.plus(robotToMechanism.inverse()))
                .orElseGet(() -> mechanismPose.plus(getLastRobotToMechanism().inverse()));
    }
}
