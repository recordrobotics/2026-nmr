package frc.robot.utils.camera;

import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionClass;
import frc.robot.utils.camera.positioned.poseestimation.PoseEstimationCamera;

/**
 * Substitutes IO based on simulation settings
 */
public final class Cameras {

    private Cameras() {}

    /**
     * Creates a MapleSimPoseEstimationCamera if in simulation mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @return The PoseEstimationCamera
     */
    public static PoseEstimationCamera createMapleSimPoseEstimationCamera(String name, PhysicalCamera physicalCamera) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new IllegalStateException("MapleSimCamera can only be created in simulation.");
        }

        return new frc.robot.utils.camera.positioned.poseestimation.io.MapleSimCamera(
                name,
                physicalCamera,
                Constants.Vision.VISION_SIMULATION_MODE == Constants.Vision.VisionSimulationMode.MAPLE_NOISE);
    }

    /**
     * Creates a MapleSimObjectDetectionCamera if in simulation mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param detectionClassMap The map of detection classes
     * @return The ObjectDetectionCamera
     */
    public static ObjectDetectionCamera createMapleSimObjectDetectionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d toCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new IllegalStateException("MapleSimCamera can only be created in simulation.");
        }

        return new frc.robot.utils.camera.positioned.objectdetection.io.MapleSimCamera(
                name, physicalCamera, toCamera, detectionClassMap);
    }

    /**
     * Creates a PhotonVisionPoseEstimationCamera or MapleSimPoseEstimationCamera based on simulation mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @return The PoseEstimationCamera
     */
    public static PoseEstimationCamera createPhotonVisionPoseEstimationCamera(
            String name, PhysicalCamera physicalCamera, Transform3d toCamera) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                || Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
            return new frc.robot.utils.camera.positioned.poseestimation.io.PhotonVisionCamera(
                    name, physicalCamera, toCamera, Constants.Vision.PHOTON_VISION_SIM_PERFORMANCE_MODE);
        }

        return createMapleSimPoseEstimationCamera(name, physicalCamera);
    }

    /**
     * Creates a PhotonVisionObjectDetectionCamera or MapleSimObjectDetectionCamera based on simulation mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param detectionClassMap The map of detection classes
     * @return The ObjectDetectionCamera
     */
    public static ObjectDetectionCamera createPhotonVisionObjectDetectionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d toCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                || Constants.Vision.OBJECT_DETECTION_SIMULATION_MODE
                        == Constants.Vision.ObjectDetectionSimulationMode.PHOTONVISION) {
            return new frc.robot.utils.camera.positioned.objectdetection.io.PhotonVisionCamera(
                    name, physicalCamera, toCamera, detectionClassMap);
        }

        return createMapleSimObjectDetectionCamera(name, physicalCamera, toCamera, detectionClassMap);
    }

    /**
     * Creates a LimelightPoseEstimationCamera if in real mode, PhotonVisionPoseEstimationCamera if in Photon sim mode,
     * or MapleSimPoseEstimationCamera if in MapleSim mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @return The PoseEstimationCamera
     */
    public static PoseEstimationCamera createLimelightPoseEstimationCamera(
            String name, PhysicalCamera physicalCamera, Transform3d toCamera) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            return new frc.robot.utils.camera.positioned.poseestimation.io.LimelightCamera(
                    name, physicalCamera, toCamera);
        } else if (Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
            return createPhotonVisionPoseEstimationCamera(name, physicalCamera, toCamera);
        }

        return createMapleSimPoseEstimationCamera(name, physicalCamera);
    }

    /**
     * Creates a LimelightObjectDetectionCamera if in real mode, PhotonVisionObjectDetectionCamera if in external photonvision mode,
     * or MapleSimObjectDetectionCamera if in MapleSim mode.
     * @param name The name of the camera
     * @param physicalCamera The physical camera
     * @param toCamera The transform from either the mechanism or robot to the camera, depending on if the camera will be moving.
     * @param detectionClassMap The map of detection classes
     * @return The ObjectDetectionCamera
     */
    public static ObjectDetectionCamera createLimelightObjectDetectionCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d toCamera,
            ImmutableSortedMap<Integer, ObjectDetectionClass> detectionClassMap) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            return new frc.robot.utils.camera.positioned.objectdetection.io.LimelightCamera(
                    name, physicalCamera, toCamera, detectionClassMap);
        } else if (Constants.Vision.OBJECT_DETECTION_SIMULATION_MODE
                == Constants.Vision.ObjectDetectionSimulationMode.PHOTONVISION) {
            return createPhotonVisionObjectDetectionCamera(name, physicalCamera, toCamera, detectionClassMap);
        }

        return createMapleSimObjectDetectionCamera(name, physicalCamera, toCamera, detectionClassMap);
    }
}
