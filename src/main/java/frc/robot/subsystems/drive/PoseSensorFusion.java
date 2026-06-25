package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.RobotToMechanismUpdate;
import frc.robot.subsystems.io.real.ImuPigeon2;
import frc.robot.subsystems.io.sim.ImuSimPigeon2;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.IndependentSwervePoseEstimator;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PositionedSubsystem.PositionStatus;
import frc.robot.utils.camera.Cameras;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.positioned.PositionedCamera.DynamicPositionMode;
import frc.robot.utils.camera.positioned.poseestimation.PoseEstimationCamera;
import java.util.EnumSet;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Subsystem for fusing multiple pose sensors (odometry, IMU, vision) into a single robot pose estimation
 */
public final class PoseSensorFusion extends ManagedSubsystemBase {

    public static final LoggedDashboardChooser<RTCMode> rtcModeChooser = new LoggedDashboardChooser<>("Camera/RTCMode");

    /**
     * The maximum standard deviation for a vision measurement (used to indicate an untrusted measurement)
     */
    public static final double MAX_MEASUREMENT_STD_DEVS = 9_999_999;

    /**
     * The standard deviation to use for the independent swerve pose estimator corrections
     */
    private static final double ISPE_STD_DEV = 0.7;

    private static final int[] BLUE_HUB_TAG_FILTER = new int[] {18, 19, 20, 21, 24, 25, 26, 27};

    private static final int[] RED_HUB_TAG_FILTER = new int[] {2, 3, 4, 5, 8, 9, 10, 11};

    public enum RTCMode {
        OFF(Pose2d.kZero),
        CORNER_DEPOT_SIDE_FORWARD(new Pose2d(0.439, 7.614, Rotation2d.kZero)),
        CORNER_DEPOT_SIDE_RIGHT(new Pose2d(0.439, 7.614, Rotation2d.kCW_90deg)),
        CORNER_DEPOT_SIDE_BACK(new Pose2d(0.439, 7.614, Rotation2d.k180deg)),
        CORNER_DEPOT_SIDE_LEFT(new Pose2d(0.439, 7.614, Rotation2d.kCCW_90deg)),
        CORNER_OUTPOST_SIDE_FORWARD(new Pose2d(0.439, FlippingUtil.fieldSizeY - 7.614, Rotation2d.kZero)),
        CORNER_OUTPOST_SIDE_RIGHT(new Pose2d(0.439, FlippingUtil.fieldSizeY - 7.614, Rotation2d.kCW_90deg)),
        CORNER_OUTPOST_SIDE_BACK(new Pose2d(0.439, FlippingUtil.fieldSizeY - 7.614, Rotation2d.k180deg)),
        CORNER_OUTPOST_SIDE_LEFT(new Pose2d(0.439, FlippingUtil.fieldSizeY - 7.614, Rotation2d.kCCW_90deg)),
        APPLY(Pose2d.kZero);

        private final Pose2d bluePose;
        private final Pose2d redPose;

        private RTCMode(Pose2d pose) {
            this.bluePose = pose;
            this.redPose = FlippingUtil.flipFieldPose(pose);
        }

        public Pose2d getPose() {
            return DriverStationUtils.getCurrentAlliance() == Alliance.Red ? redPose : bluePose;
        }
    }

    private static final LoggedNetworkBoolean filterTagsToggle = new LoggedNetworkBoolean("Camera/FilterTags", false);
    private static final LoggedNetworkBoolean prioritizeTurretToggle =
            new LoggedNetworkBoolean("Camera/PrioritizeTurret", true);

    /**
     * The IMU sensor used for orientation and acceleration data
     */
    public final Imu imu;

    /**
     * The swerve drive pose estimator used for fusing odometry and vision measurements
     */
    private SwerveDrivePoseEstimator poseFilter;
    /**
     * The independent swerve pose estimator used for better odometry when no vision is available
     */
    private IndependentSwervePoseEstimator independentPoseEstimator;

    private final PoseEstimationCamera turretCamera = Cameras.createLimelightPoseEstimationCamera(
                    Constants.Vision.TURRET_NAME,
                    PhysicalCamera.LIMELIGHT_4,
                    Constants.Vision.MECHANISM_TO_CAMERA_TURRET)
            .setUnconstrainedMaxDistance(0)
            .setDynamicPositionMode(DynamicPositionMode.MECHANISM_TO_CAMERA)
            .setForceUnconstrainedWhenDisabled(true);

    /**
     * The cameras used for vision measurements
     */
    private final Set<PoseEstimationCamera> cameras = Set.of(
            Cameras.createLimelightPoseEstimationCamera(
                            Constants.Vision.RIGHT_FRONT_NAME,
                            PhysicalCamera.LIMELIGHT_3G,
                            Constants.Vision.ROBOT_TO_CAMERA_RIGHT_FRONT)
                    .setForceUnconstrained(true),
            Cameras.createPhotonVisionPoseEstimationCamera(
                            Constants.Vision.LEFT_BACK_NAME,
                            PhysicalCamera.SVPRO_GLOBAL_SHUTTER,
                            Constants.Vision.ROBOT_TO_CAMERA_LEFT_BACK)
                    .setForceUnconstrained(true),
            Cameras.createPhotonVisionPoseEstimationCamera(
                            Constants.Vision.RIGHT_BACK_NAME,
                            PhysicalCamera.SVPRO_GLOBAL_SHUTTER,
                            Constants.Vision.ROBOT_TO_CAMERA_RIGHT_BACK)
                    .setForceUnconstrained(true),
            turretCamera);

    /**
     * The deferred pose estimations to be added at the end of the calculation phase
     */
    private ConcurrentSkipListSet<DeferredPoseEstimation> deferredPoseEstimations =
            new ConcurrentSkipListSet<>((a, b) -> Double.compare(a.timestampSeconds, b.timestampSeconds));

    /**
     * Executor service for asynchronous calculations
     */
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    /**
     * Future for the current calculation task
     */
    private Future<?> calculationFuture = null;

    /**
     * Last update timestamp
     */
    private double updateTimestamp;
    /**
     * Last update nav angle
     */
    private Rotation2d updateNav = Rotation2d.kZero;
    /**
     * Last update swerve module positions
     */
    private SwerveModulePosition[] updatePositions;

    /**
     * The TXTY ID to use for vision measurements (-1 for disabled)
     */
    private int targetTXTYId = -1;

    private Rotation2d lastRawNavAngle = Rotation2d.kZero;

    private double lastTurretVisionTime = 0;

    /**
     * Creates a new PoseSensorFusion subsystem
     * @param initialPose the initial pose of the robot on the field
     */
    public PoseSensorFusion(Pose2d initialPose) {
        imu = new Imu(
                Constants.RobotState.getMode() == Mode.REAL
                        ? new ImuPigeon2()
                        : new ImuSimPigeon2(RobotContainer.drivetrain
                                .getSwerveDriveSimulation()
                                .getGyroSimulation()));

        lastRawNavAngle = imu.isConnected() ? imu.getYaw() : Rotation2d.kZero;

        poseFilter = new SwerveDrivePoseEstimator(
                RobotContainer.drivetrain.getKinematics(),
                lastRawNavAngle,
                RobotContainer.drivetrain.getModulePositions(),
                initialPose);

        independentPoseEstimator = new IndependentSwervePoseEstimator(
                getEstimatedPosition(),
                RobotContainer.drivetrain.getModules(),
                RobotContainer.drivetrain.getKinematics().getModules());

        EnumSet.allOf(RTCMode.class).forEach(v -> rtcModeChooser.addOption(v.name(), v));
        rtcModeChooser.addDefaultOption(RTCMode.OFF.name(), RTCMode.OFF);
    }

    public record DeferredPoseEstimation(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

    /**
     * Adds a vision measurement to be processed at the end of the calculation phase
     * @param visionRobotPoseMeters the vision-based robot pose measurement
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        deferredPoseEstimations.add(
                new DeferredPoseEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    }

    /**
     * Starts the asynchronous calculation phase
     */
    public void startCalculation() {
        deferredPoseEstimations.clear();
        calculationFuture = executor.submit(this::performCalculation);
    }

    /**
     * Waits for the end of the calculation and applies all updates
     */
    public void endCalculation() {
        if (updatePositions != null) {
            poseFilter.updateWithTime(updateTimestamp, updateNav, updatePositions);
        }

        if (calculationFuture != null) {
            try {
                calculationFuture.get();
            } catch (ExecutionException e) {
                ConsoleLogger.logError(e);
            } catch (InterruptedException e) {
                ConsoleLogger.logError(e);
                Thread.currentThread().interrupt();
            }
        }

        while (!deferredPoseEstimations.isEmpty()) {
            DeferredPoseEstimation estimation = deferredPoseEstimations.pollFirst();
            poseFilter.addVisionMeasurement(
                    estimation.visionRobotPoseMeters, estimation.timestampSeconds, estimation.visionMeasurementStdDevs);
        }

        logValues();
    }

    public void recordMatchReplay() {
        cameras.stream().forEach(GenericCamera::recordMatchReplay);
    }

    /**
     * Logs values to the logger
     */
    private void logValues() {
        cameras.stream().forEach(GenericCamera::logValues);

        Logger.recordOutput("SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
        Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
        Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());

        Logger.recordOutput("TargetTXTYId", targetTXTYId);
    }

    @Override
    public void periodicManaged() {
        /* logic is asynchronous */
    }

    /**
     * Performs the asynchronous calculation for the pose sensor fusion
     */
    public void performCalculation() {
        updateTimestamp = Timer.getTimestamp();

        SwerveModulePosition[] positions = RobotContainer.drivetrain.getModulePositions();

        if (imu.isConnected() && !imu.isFaulted()) {
            updateNav = imu.getYaw();
        } else if (updatePositions != null) {
            SwerveModulePosition[] deltas = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                deltas[i] = new SwerveModulePosition(
                        positions[i].distanceMeters - updatePositions[i].distanceMeters, positions[i].angle);
            }

            Twist2d twist = RobotContainer.drivetrain.getKinematics().toTwist2d(deltas);
            updateNav = lastRawNavAngle.plus(new Rotation2d(twist.dtheta));
        }

        lastRawNavAngle = updateNav;
        updatePositions = positions;

        if (RobotContainer.turret.getPositionStatus() == PositionStatus.KNOWN) {
            RobotToMechanismUpdate robotToMechanismUpdate = RobotContainer.turret.getRobotToMechanism();
            turretCamera.updateRobotToMechanism(
                    robotToMechanismUpdate.robotToMechanism(), robotToMechanismUpdate.timestamp());
        }

        cameras.stream().forEach(GenericCamera::periodic);

        /* perform any multi-camera logic like txty selection (currently manual) */

        RTCMode rtcMode = rtcModeChooser.get();
        Pose2d currentEstimate = getEstimatedPosition();

        if (prioritizeTurretToggle.get() && turretCamera.hasVision()) {
            lastTurretVisionTime = Timer.getTimestamp();
        }

        if (prioritizeTurretToggle.get() && Timer.getTimestamp() - lastTurretVisionTime < 0.1) {
            cameras.stream().forEach(camera -> {
                if (camera != turretCamera) {
                    camera.setIgnore(true);
                }
            });
        } else {
            cameras.stream().forEach(camera -> {
                if (camera != turretCamera) {
                    camera.setIgnore(false);
                }
            });
        }

        cameras.stream().forEach(camera -> {
            if (rtcMode != null && rtcMode != RTCMode.OFF && rtcMode != RTCMode.APPLY) {
                camera.setComputeRobotToCamera(true, new Pose3d(rtcMode.getPose()));
            } else if (camera.isComputingRobotToCamera()) {
                if (rtcMode == RTCMode.APPLY) {
                    camera.applyCalculatedRobotToCamera();
                }
                camera.setComputeRobotToCamera(false);
            }

            if (filterTagsToggle.get()) {
                camera.setFilter(
                        DriverStationUtils.getCurrentAlliance() == Alliance.Red
                                ? RED_HUB_TAG_FILTER
                                : BLUE_HUB_TAG_FILTER);
            } else {
                camera.setFilter(Constants.Game.ALL_TAGS);
            }

            if ((camera.hasRobotToMechanism()
                            || camera.getDynamicPositionMode() != DynamicPositionMode.MECHANISM_TO_CAMERA)
                    && !camera.isIgnored()) {
                camera.addVisionMeasurements(this, currentEstimate, targetTXTYId);
            }
        });

        updateIndependentPoseEstimator();
    }

    /**
     * Updates the independent pose estimator and uses it to correct the main pose estimator when no vision is available
     */
    private void updateIndependentPoseEstimator() {
        Pose2d estimatedPose = getEstimatedPosition();
        if (cameras.stream().anyMatch(v -> v.hasVision())) {
            // when vision is correcting the pose, have that override the independent pose estimator
            independentPoseEstimator.reset(estimatedPose);
            independentPoseEstimator.update(estimatedPose.getRotation());
        } else {
            independentPoseEstimator.update(estimatedPose.getRotation());

            // when no vision use independent pose estimator to correct pose
            addVisionMeasurement(
                    independentPoseEstimator.getEstimatedRobotPose(),
                    Timer.getTimestamp(),
                    VecBuilder.fill(ISPE_STD_DEV, ISPE_STD_DEV, MAX_MEASUREMENT_STD_DEVS));
        }
    }

    /**
     * Gets the estimated position of the robot on the field at the current time
     * @return the estimated position of the robot on the field
     */
    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    @AutoLogLevel(key = "Odometry/Robot", level = Level.REAL)
    public Pose3d getEstimatedPosition3d() {
        Pose2d pose2d = getEstimatedPosition();
        return new Pose3d(
                new Translation3d(pose2d.getTranslation()),
                new Rotation3d(
                        RobotContainer.poseSensorFusion.imu.getRoll().getRadians(),
                        RobotContainer.poseSensorFusion.imu.getPitch().getRadians(),
                        pose2d.getRotation().getRadians()));
    }

    /**
     * Gets the estimated position at a specific timestamp or an empty optional if the timestamp is out of range
     * @param timestamp the timestamp to get the estimated position at (see {@link Timer#getTimestamp()})
     * @return an optional containing the estimated position at the specified timestamp, or empty if out of range
     */
    public Optional<Pose2d> getEstimatedPositionAt(double timestamp) {
        return poseFilter.sampleAt(timestamp);
    }

    /** Resets the robot's pose to the specified pose (if in simulation also resets simulation pose) */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(lastRawNavAngle, RobotContainer.drivetrain.getModulePositions(), pose);
        independentPoseEstimator.reset(pose);
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(pose);
        }
    }

    /**
     * Aligns the robot's rotation to face the robot in the same direction as the driver station
     */
    public void alignRotationWithDriverStation() {
        Pose2d current = getEstimatedPosition();
        setToPose(new Pose2d(
                current.getTranslation(),
                DriverStationUtils.getCurrentAlliance() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d(0)));
    }

    /**
     * Closes the subsystem and releases any resources
     */
    @Override
    public void close() {
        imu.close();
    }
}
