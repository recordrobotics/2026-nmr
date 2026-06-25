package frc.robot.utils.camera.positioned.poseestimation.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Vision.PhotonVisionSimPerformanceMode;
import frc.robot.RobotContainer;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import frc.robot.utils.camera.positioned.poseestimation.PoseEstimationCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision protocol camera for pose estimation.
 */
public class PhotonVisionCamera extends PoseEstimationCamera {

    private static final int THROTTLE_FPS = 3;

    /**
     * The PhotonCamera instance.
     */
    private final PhotonCamera camera;

    private final PhotonCameraSim cameraSim;

    /**
     * The PhotonPoseEstimator instance.
     */
    private final PhotonPoseEstimator photonEstimator;

    /**
     * Constructs a PhotonVisionCamera with the given name, physical camera, and robot-to-camera transform.
     * @param name The name of the camera.
     * @param physicalCamera The physical camera type.
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param performanceMode The performance mode for the simulation.
     */
    public PhotonVisionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d toCamera,
            PhotonVisionSimPerformanceMode performanceMode) {
        super(name, physicalCamera, toCamera);

        camera = new PhotonCamera(name);

        photonEstimator = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT,
                convertRobotToCamera(
                        getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                                ? getRobotToCamera()
                                : getMechanismToCamera()));

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(
                    physicalCamera.getDetectorWidth(),
                    physicalCamera.getDetectorHeight(),
                    Rotation2d.fromDegrees(physicalCamera.fov));
            cameraProp.setCalibError(physicalCamera.pxError, physicalCamera.pxErrorStdDev);
            cameraProp.setFPS(physicalCamera.fps);
            cameraProp.setAvgLatencyMs(physicalCamera.latencyMs);
            cameraProp.setLatencyStdDevMs(physicalCamera.latencyStdDevMs);

            cameraSim = new PhotonCameraSim(camera, cameraProp);

            switch (performanceMode) {
                case FAST:
                    cameraSim.enableDrawWireframe(false);
                    cameraSim.enableRawStream(false);
                    cameraSim.enableProcessedStream(false);
                    break;
                case STREAMED:
                    cameraSim.enableDrawWireframe(false);
                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    break;
                case WIREFRAME:
                    cameraSim.enableDrawWireframe(true);
                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    break;
            }

            cameraSim.setMaxSightRange(physicalCamera.maxSightRange);
            cameraSim.setMinTargetAreaPixels(physicalCamera.minTargetAreaPixels);

            RobotContainer.visionSim.addCamera(cameraSim, convertRobotToCamera(getRobotToCamera()));
        } else {
            cameraSim = null;
        }
    }

    private static Transform3d convertRobotToCamera(Transform3d robotToCamera) {
        return new Transform3d(
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                new Rotation3d(
                        robotToCamera.getRotation().getX(),
                        -robotToCamera.getRotation().getY(),
                        robotToCamera.getRotation().getZ()));
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
    public void setFilter(int[] filter) {
        // todo
    }

    @Override
    public void recordMatchReplay() {
        // not supported
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    @Override
    public List<CameraPoseEstimate> makeEstimates() {
        if (!isConnected()) {
            return List.of();
        }

        addHeadingDataToEstimator();
        return getPhotonVisionResults();
    }

    /**
     * Adds the current robot heading data to the PhotonPoseEstimator.
     */
    private void addHeadingDataToEstimator() {
        double timestamp = Timer.getTimestamp();
        Rotation3d heading = new Rotation3d(
                use3DRotation() ? RobotContainer.poseSensorFusion.imu.getRoll().getRadians() : 0,
                use3DRotation() ? RobotContainer.poseSensorFusion.imu.getPitch().getRadians() : 0,
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians());

        photonEstimator.addHeadingData(
                timestamp,
                heading.plus(getLastRobotToMechanism().getRotation())
                        .toRotation2d() /* photonvision ignores roll and pitch but does it matter??? im confused test on actual robot */);
        photonEstimator.setRobotToCameraTransform(convertRobotToCamera((getDynamicPositionMode()
                                == DynamicPositionMode.ROBOT_TO_CAMERA
                        ? getRobotToCamera()
                        : getMechanismToCamera())
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(heading.getX(), heading.getY(), 0)))));

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.visionSim.adjustCamera(cameraSim, convertRobotToCamera(getRobotToCamera()));
        }

        if (DriverStation.isEnabled() || isForcingUnthrottled()) {
            camera.setFPSLimit(-1);
        } else {
            camera.setFPSLimit(THROTTLE_FPS);
        }
    }

    /**
     * Gets the pose estimates from PhotonVision results.
     * @return A list of camera pose estimates.
     */
    private List<CameraPoseEstimate> getPhotonVisionResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return List.of();
        }

        List<CameraPoseEstimate> measurements = new ArrayList<>();

        Optional<Matrix<N3, N3>> cameraMatrix = camera.getCameraMatrix();
        Optional<Matrix<N8, N1>> distortionCoeffs = camera.getDistCoeffs();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            photonEstimator.setRobotToCameraTransform(convertRobotToCamera(
                    getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                            ? getRobotToCameraAt(result.getTimestampSeconds()).orElse(getRobotToCamera())
                            : getMechanismToCamera()));

            Optional<EstimatedRobotPose> unconstrainedEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (unconstrainedEst.isEmpty()) {
                unconstrainedEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            Optional<EstimatedRobotPose> constrainedEst;
            if (unconstrainedEst.isPresent() && cameraMatrix.isPresent() && distortionCoeffs.isPresent()) {
                Logger.recordOutput(getName() + "/" + "TEMP1", unconstrainedEst.get().estimatedPose);
                constrainedEst = photonEstimator.estimateConstrainedSolvepnpPose(
                        result,
                        cameraMatrix.get(),
                        distortionCoeffs.get(),
                        unconstrainedEst.get().estimatedPose,
                        false,
                        1.0);
                Logger.recordOutput(getName() + "/" + "TEMP2", constrainedEst.get().estimatedPose);
            } else {
                constrainedEst = Optional.empty();
            }

            Optional<EstimatedRobotPose> txtyEst = photonEstimator.estimatePnpDistanceTrigSolvePose(result);
            double bestTargetDist = result.getBestTarget()
                    .getBestCameraToTarget()
                    .getTranslation()
                    .getNorm();

            if (unconstrainedEst.isPresent()
                    && !unconstrainedEst.get().targetsUsed.isEmpty()) {
                measurements.add(getCameraPoseEstimateFromPhotonEstimates(
                        unconstrainedEst.get(),
                        constrainedEst,
                        txtyEst.map(v -> new TXTYMeasurement(
                                (getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                                                ? v.estimatedPose
                                                : calculateRobotPose(v.estimatedPose, v.timestampSeconds))
                                        .toPose2d(),
                                bestTargetDist,
                                v.targetsUsed.get(0).fiducialId))));
            }
        }

        return measurements;
    }

    /**
     * Converts PhotonVision estimated robot poses to a CameraPoseEstimate.
     * @param unconstrainedEst The unconstrained estimated robot pose.
     * @param constrainedEst The constrained estimated robot pose.
     * @param txtyEst The TXTY estimated robot pose.
     * @return A CameraPoseEstimate object representing the combined pose estimates.
     */
    @SuppressWarnings("java:S3553") // input is from output of another function
    private CameraPoseEstimate getCameraPoseEstimateFromPhotonEstimates(
            EstimatedRobotPose unconstrainedEst,
            Optional<EstimatedRobotPose> constrainedEst,
            Optional<TXTYMeasurement> txtyEst) {

        int tagCount = unconstrainedEst.targetsUsed.size();

        double avgTagDist = 0;
        double avgTagArea = 0;

        for (PhotonTrackedTarget target : unconstrainedEst.targetsUsed) {
            double dist = target.getBestCameraToTarget().getTranslation().getNorm();
            avgTagDist += dist;
            avgTagArea += target.getArea();
        }
        avgTagDist /= tagCount;
        avgTagArea /= tagCount;

        return new CameraPoseEstimate(
                (getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                        ? unconstrainedEst.estimatedPose
                        : calculateRobotPose(unconstrainedEst.estimatedPose, unconstrainedEst.timestampSeconds)),
                constrainedEst.map(e -> (getDynamicPositionMode() == DynamicPositionMode.ROBOT_TO_CAMERA
                                ? e.estimatedPose
                                : calculateRobotPose(e.estimatedPose, e.timestampSeconds))
                        .toPose2d()),
                txtyEst.map(List::of).orElse(List.of()),
                unconstrainedEst.timestampSeconds,
                getPhysicalCamera().latencyMs,
                tagCount,
                avgTagDist,
                avgTagArea);
    }
}
