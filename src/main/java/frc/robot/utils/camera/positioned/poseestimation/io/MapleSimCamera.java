package frc.robot.utils.camera.positioned.poseestimation.io;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate;
import frc.robot.utils.camera.positioned.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import frc.robot.utils.camera.positioned.poseestimation.PoseEstimationCamera;
import java.util.List;
import java.util.Optional;

/**
 * MapleSim simulated camera for pose estimation.
 */
public class MapleSimCamera extends PoseEstimationCamera {

    /**
     * The AprilTags used in the MapleSim simulation.
     */
    private static final List<AprilTag> MAPLE_SIM_TAGS = Constants.Game.APRILTAG_LAYOUT.getTags();

    /**
     * The standard deviation of the noise added to the MapleSim camera pose estimates.
     */
    private static final double MAPLE_SIM_STDDEV = 0.001;
    /**
     * The average distance to tags in the MapleSim simulation.
     */
    private static final double MAPLE_SIM_TAG_DIST = 6;
    /**
     * The average area of tags in the MapleSim simulation.
     */
    private static final double MAPLE_SIM_TAG_AREA = 2.0;

    private boolean addNoise;

    /**
     * Constructs a MapleSimCamera with the given name and physical camera.
     * @param name The name of the camera.
     * @param physicalCamera The physical camera type.
     */
    public MapleSimCamera(String name, PhysicalCamera physicalCamera, boolean addNoise) {
        super(name, physicalCamera, Transform3d.kZero);
        this.addNoise = addNoise;
    }

    public boolean hasNoise() {
        return addNoise;
    }

    public void setAddNoise(boolean addNoise) {
        this.addNoise = addNoise;
    }

    @Override
    public void setFilter(int[] filter) {
        // noop
    }

    @Override
    public void recordMatchReplay() {
        // not supported
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

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    @Override
    public List<CameraPoseEstimate> makeEstimates() {
        Pose3d maplePose = RobotContainer.model.getRobot();
        if (addNoise) {
            maplePose = SimpleMath.poseNoise(maplePose, MAPLE_SIM_STDDEV, MAPLE_SIM_STDDEV);
        }

        final Pose3d finalMaplePose = maplePose;

        return List.of(new CameraPoseEstimate(
                finalMaplePose,
                Optional.of(finalMaplePose.toPose2d()),
                MAPLE_SIM_TAGS.stream()
                        .map(tag -> new TXTYMeasurement(
                                finalMaplePose.toPose2d(),
                                new Translation3d(finalMaplePose.toPose2d().getTranslation())
                                        .getDistance(tag.pose.getTranslation()),
                                tag.ID))
                        .toList(),
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(getPhysicalCamera().latencyMs),
                MAPLE_SIM_TAGS.size(),
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA));
    }
}
