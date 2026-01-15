package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.google.common.primitives.ImmutableIntArray;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.field.FieldIntersection;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.Logger;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends ManagedSubsystemBase {

    public interface MechanismModel {
        int getPoseCount();

        void updatePoses(Pose3d[] poses, int i);
    }

    public record Pose3dObject(Pose3d pose, int identity) {}

    private static FuelManager fuelManager;

    public final IntakeModel intakeModel = new IntakeModel();
    public final ShooterModel shooterModel = new ShooterModel();
    public final ClimberModel climberModel = new ClimberModel();

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses =
            new Pose3d[intakeModel.getPoseCount() + shooterModel.getPoseCount() + climberModel.getPoseCount()];

    public RobotModel() {
        periodicManaged();
    }

    public static FuelManager getFuelManager() {
        if (fuelManager == null && Constants.RobotState.getMode() != Mode.REAL) {
            fuelManager = new FuelManager();
            fuelManager
                    .init(); // separate from constructor to avoid infinite recursion with ManagedFuelNode constructor
        }
        return fuelManager;
    }

    @Override
    public void periodicManaged() {
        updatePoses(intakeModel, shooterModel, climberModel);
        if (Constants.RobotState.getMode() != Mode.REAL) {
            fuelManager.update();
        }

        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            Logger.recordOutput("IGamePositions", IGamePosition.aggregatePositions());
            FieldIntersection.logAllInstances();
        }

        // RSL state is set by FPGA and therefore is not simulated by default
        // Only override it in simulation, replay mode should use the logged value
        // from the real robot.
        if (Constants.RobotState.getMode() == Mode.SIM) {
            NetworkTableInstance.getDefault()
                    .getEntry("/AdvantageKit/SystemStats/RSLState")
                    .setBoolean(getSimulatedRSLState());
        }
    }

    private boolean getSimulatedRSLState() {
        if (!DriverStation.isEnabled()) {
            return true;
        }

        return (Timer.getTimestamp() / 0.2) % 2 < 1; // toggle every 0.2 seconds
    }

    private void updatePoses(MechanismModel... mechanismModels) {
        int i = 0;
        for (MechanismModel mechanismModel : mechanismModels) {
            if (i >= mechanismPoses.length) {
                ConsoleLogger.logError("RobotModel.updatePoses: too many mechanisms");
                break;
            }

            mechanismModel.updatePoses(mechanismPoses, i);
            i += mechanismModel.getPoseCount();
        }
    }

    @AutoLogLevel(level = Level.SIM)
    public Pose3d[] getFuelPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> arenaFuelPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel");
            arenaFuelPoses.addAll(fuelManager.getFuelPoses());
            return arenaFuelPoses.toArray(Pose3d[]::new);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    public Pose3dObject[] getFuelObjects() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3dObject> arenaFuelObjects = SimulatedArena.getInstance().getGamePiecesByType("Fuel").stream()
                    .map(fuelObject -> new Pose3dObject(fuelObject.getPose3d(), System.identityHashCode(fuelObject)))
                    .collect(Collectors.toList());
            arenaFuelObjects.addAll(fuelManager.getFuelObjects());
            return arenaFuelObjects.toArray(Pose3dObject[]::new);
        } else {
            return new Pose3dObject[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose3d getRobot() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return RobotContainer.drivetrain.getLastSimPose3d();
        } else {
            return Pose3d.kZero;
        }
    }

    public static class IntakeModel implements MechanismModel {
        public static final int POSE_COUNT = 2;
        private static final Translation3d SHAFT_ORIGIN = new Translation3d(-0.285662, 0, 0.274755);
        private static final ArmGeometryPoint[] ARM_GEOMETRY = new ArmGeometryPoint[] {
            new ArmGeometryPoint(Units.degreesToRadians(53.532698), Units.degreesToRadians(-51.8490194205), 0.314003),
            new ArmGeometryPoint(Units.degreesToRadians(17.329479), Units.degreesToRadians(-19.7844139869), 0.356156),
            new ArmGeometryPoint(Units.degreesToRadians(0), Units.degreesToRadians(-1.19101124384), 0.352317)
        };
        private static final double HOPPER_TO_AXLE_DISTANCE_METERS = 0.054722;

        private double angleRadians;
        private double hopperExtensionMeters;

        public void update(double newAngleRadians) {
            angleRadians = newAngleRadians;
            hopperExtensionMeters = Math.max(
                    hopperExtensionMeters, ArmGeometryPoint.calculateHopperExtension(angleRadians, ARM_GEOMETRY));
        }

        public void resetHopperExtension() {
            hopperExtensionMeters = 0;
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(SHAFT_ORIGIN, new Rotation3d(0, angleRadians, 0));
            poses[i + 1] = new Pose3d(-hopperExtensionMeters, 0, 0, Rotation3d.kZero);
        }

        private record ArmGeometryPoint(double lowestArmAngleRadians, double angleOffset, double length) {
            private static ArmGeometryPoint findCurrentPoint(double armAngle, ArmGeometryPoint[] geometry) {
                for (ArmGeometryPoint point : geometry) {
                    if (armAngle >= point.lowestArmAngleRadians) {
                        return point;
                    }
                }
                return geometry[
                        geometry.length - 1]; // if we are below the lowest point, just use the lowest point's geometry
            }

            private static double calculateHopperExtension(double armAngle, ArmGeometryPoint[] geometry) {
                ArmGeometryPoint currentPoint = findCurrentPoint(armAngle, geometry);
                return Math.max(
                        0,
                        currentPoint.length * Math.cos(armAngle + currentPoint.angleOffset)
                                - HOPPER_TO_AXLE_DISTANCE_METERS);
            }
        }
    }

    public static class ShooterModel implements MechanismModel {
        public static final int POSE_COUNT = 2;
        private static final Translation3d TURRET_ORIGIN = new Translation3d(0.12715, 0.12715, 0);
        private static final Translation3d HOOD_LOCAL_ORIGIN = new Translation3d(0.2105, 0, 0.4556);

        private double turretAngleRadians;
        private double hoodAngleRadians;

        public void updateTurret(double newTurretAngleRadians) {
            turretAngleRadians = newTurretAngleRadians;
        }

        public void updateHood(double newHoodAngleRadians) {
            hoodAngleRadians = newHoodAngleRadians;
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(TURRET_ORIGIN, new Rotation3d(0, 0, turretAngleRadians));
            poses[i + 1] = poses[i].transformBy(new Transform3d(
                    Translation3d.kZero.rotateAround(HOOD_LOCAL_ORIGIN, new Rotation3d(0, -hoodAngleRadians, 0)),
                    new Rotation3d(0, -hoodAngleRadians, 0)));
        }

        public Translation3d getShooterFuelReleasePosition() {
            Translation3d shooterBallStart = new Translation3d(0.127000, 0.127000, 0.358781);
            Translation3d shooterOrigin = new Translation3d(0.210586, 0.239469, 0.455638)
                    .rotateAround(
                            shooterBallStart,
                            new Rotation3d(Rotation2d.fromRotations(RobotContainer.turret.getPositionRotations())));
            Translation3d axisOfRotation = new Translation3d(0, 1, 0)
                    .rotateBy(new Rotation3d(Rotation2d.fromRotations(RobotContainer.turret.getPositionRotations())));
            return SimpleMath.rotateAroundWithRadius(
                            new Pose3d(shooterBallStart, Rotation3d.kZero),
                            shooterOrigin,
                            axisOfRotation,
                            0.113106,
                            Math.PI - RobotContainer.shooter.getHoodAngle() - Math.PI / 4)
                    .getTranslation();
        }

        public Translation3d getShooterHoodPosition() {
            Translation3d shooterHoodStart = new Translation3d(0.024704, 0.127000, 0.526709);
            Translation3d shooterBallStart = new Translation3d(0.127000, 0.127000, 0.358781);
            Translation3d shooterOrigin = new Translation3d(0.210586, 0.239469, 0.455638);
            return shooterHoodStart
                    .rotateAround(
                            shooterOrigin,
                            new Rotation3d(
                                    0,
                                    Constants.Shooter.HOOD_MAX_POSITION_RADIANS - RobotContainer.shooter.getHoodAngle(),
                                    0))
                    .rotateAround(
                            shooterBallStart,
                            new Rotation3d(Rotation2d.fromRotations(RobotContainer.turret.getPositionRotations())));
        }
    }

    public static class ClimberModel implements MechanismModel {
        public static final int POSE_COUNT = 2;
        private static final Translation3d SHAFT_ORIGIN = new Translation3d(0.292, 0, 0.5063);

        private double climberHeightMeters;
        private double shotBlockerAngle;

        public void update(double newClimberHeightMeters) {
            climberHeightMeters = newClimberHeightMeters;
            shotBlockerAngle = MathUtil.interpolate(
                    -Math.PI / 2,
                    0,
                    (climberHeightMeters - Constants.Climber.CLIMBER_SHOTBLOCKER_EXTEND_START_HEIGHT)
                            / (Constants.Climber.CLIMBER_SHOTBLOCKER_EXTEND_END_HEIGHT
                                    - Constants.Climber.CLIMBER_SHOTBLOCKER_EXTEND_START_HEIGHT));
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = new Pose3d(0, 0, climberHeightMeters, Rotation3d.kZero);
            Pose3d shotBlockerPose = Pose3d.kZero.rotateAround(SHAFT_ORIGIN, new Rotation3d(0, -shotBlockerAngle, 0));
            poses[i + 1] = new Pose3d(
                    shotBlockerPose.getTranslation().plus(new Translation3d(0, 0, climberHeightMeters)),
                    shotBlockerPose.getRotation());
        }
    }

    public static class FuelManager {

        private static final double FUEL_DIAMETER_METERS = 0.15; // 15 cm

        public enum AnimationType {
            CURVED,
            ANGULAR
        }

        public record LineSegment(Translation3d start, Translation3d end) {

            public boolean isIntersectingWithSphere(Translation3d center, double radius) {
                Translation3d d = end.minus(start);
                Translation3d f = start.minus(center);

                double a = d.dot(d);
                double b = 2 * f.dot(d);
                double c = f.dot(f) - radius * radius;

                double discriminant = b * b - 4 * a * c;
                if (discriminant < 0) {
                    return false; // no intersection
                }

                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
            }
        }

        public record FuelNode(
                Pose3d pose,
                ImmutableIntArray intakeNextNodes,
                ImmutableIntArray outtakeNextNodes,
                AnimationType outtakeAnimation) {}

        public static class FuelObject {
            private Pose3d pose;
            private Transform3d velocity = new Transform3d();

            private ManagedFuelNode currentNode;
            private Pose3d animationTargetPose;
            private double animationSpeed;

            private boolean isRotating;
            private Pose3d rotationStartPose;
            private Supplier<Translation3d> rotationCenter;
            private Supplier<Translation3d> rotationAxis;
            private DoubleSupplier rotationAngle;
            private double currentRotationAngle;
            private DoubleSupplier angularVelocitySupplier;
            private Runnable animationCompleteCallback;
            private double rotationRadius;

            private Function<FuelObject, Boolean> marker;

            public FuelObject(Pose3d initialPose, ManagedFuelNode initialNode) {
                this.pose = initialPose;
                this.currentNode = initialNode;
                this.currentNode.occupyingObject = this;
            }

            public Pose3d getPose() {
                return pose;
            }

            public void setMarker(Function<FuelObject, Boolean> marker) {
                this.marker = marker;
            }

            public void animateTo(Pose3d newPose, double speed, Runnable onComplete) {
                animationTargetPose = newPose;
                isRotating = false;
                this.animationCompleteCallback = onComplete;
                this.animationSpeed = speed;
            }

            public void rotateAround(
                    Supplier<Translation3d> rotationCenter,
                    Supplier<Translation3d> rotationAxis,
                    double radius,
                    DoubleSupplier angle,
                    DoubleSupplier angularVelocitySupplier,
                    Runnable onComplete) {
                isRotating = true;
                currentRotationAngle = 0;
                rotationStartPose =
                        SimpleMath.rotateAroundWithRadius(pose, rotationCenter.get(), rotationAxis.get(), radius, 0);
                this.rotationRadius = radius;
                this.rotationCenter = rotationCenter;
                this.rotationAxis = rotationAxis;
                this.rotationAngle = angle;
                this.angularVelocitySupplier = angularVelocitySupplier;
                this.animationCompleteCallback = onComplete;
            }

            public void update(double dt) {
                if (isRotating) {
                    double angularVelocity = angularVelocitySupplier.getAsDouble();
                    double targetAngle = rotationAngle.getAsDouble();
                    double angleToRotate = angularVelocity * dt;
                    currentRotationAngle =
                            MathUtil.clamp(currentRotationAngle + angleToRotate, -targetAngle, targetAngle);
                    Pose3d newPose = SimpleMath.rotateAroundWithRadius(
                            rotationStartPose,
                            rotationCenter.get(),
                            rotationAxis.get(),
                            rotationRadius,
                            currentRotationAngle);
                    velocity = new Transform3d(
                                    newPose.getTranslation().minus(pose.getTranslation()),
                                    newPose.getRotation().minus(pose.getRotation()))
                            .div(dt);
                    pose = newPose;
                    if (Math.abs(currentRotationAngle) >= Math.abs(targetAngle)) {
                        isRotating = false;
                        if (animationCompleteCallback != null) {
                            Runnable tempCallback = animationCompleteCallback;
                            animationCompleteCallback = null;
                            tempCallback.run();
                        }
                    }
                } else {
                    if (animationTargetPose != null) {
                        Transform3d toTarget = new Transform3d(pose, animationTargetPose);

                        velocity = toTarget.times(animationSpeed);

                        // limit velocity to prevent overshooting
                        double maxVelocity = animationSpeed / 9.0 * 2.0;
                        if (velocity.getTranslation().getNorm() > maxVelocity) {
                            velocity = new Transform3d(
                                    velocity.getTranslation()
                                            .times(maxVelocity
                                                    / velocity.getTranslation().getNorm()),
                                    velocity.getRotation());
                        }

                        pose = pose.plus(velocity.times(dt));

                        if (pose.getTranslation().getDistance(animationTargetPose.getTranslation())
                                < 0.01 * animationSpeed / 9.0) {
                            pose = animationTargetPose;
                            velocity = Transform3d.kZero;
                            if (animationCompleteCallback != null) {
                                Runnable tempCallback = animationCompleteCallback;
                                animationCompleteCallback = null;
                                animationTargetPose = null;
                                tempCallback.run();
                            }
                        }
                    } else {

                        velocity = Transform3d.kZero;
                    }
                }

                if (marker != null && marker.apply(this)) {
                    marker = null; // clear marker after it's triggered
                }
            }

            public boolean intersects(LineSegment lineSegment) {
                return lineSegment.isIntersectingWithSphere(pose.getTranslation(), FUEL_DIAMETER_METERS / 2);
            }
        }

        public static class ManagedFuelNode {
            private final FuelNode node;
            private final ImmutableIntArray intakePreviousNodes;
            private final ImmutableIntArray outtakePreviousNodes;
            private final ImmutableIntArray allForwardNodes;
            private FuelObject occupyingObject;

            public ManagedFuelNode(FuelNode node) {
                this.node = node;
                this.occupyingObject = null;

                int nodeIndex = Arrays.asList(RobotModel.getFuelManager().ROBOT_FUEL_NODES)
                                .indexOf(node)
                        + 1;
                this.intakePreviousNodes = findPreviousNodes(
                        nodeIndex, RobotModel.getFuelManager().ROBOT_FUEL_NODES, FuelNode::intakeNextNodes);
                this.outtakePreviousNodes = findPreviousNodes(
                        nodeIndex, RobotModel.getFuelManager().ROBOT_FUEL_NODES, FuelNode::outtakeNextNodes);

                this.allForwardNodes = ImmutableIntArray.builder()
                        .addAll(node.intakeNextNodes)
                        .addAll(outtakePreviousNodes)
                        .addAll(node.outtakeNextNodes)
                        .build();
            }

            public boolean isOccupied() {
                return occupyingObject != null;
            }

            public ImmutableIntArray intakePreviousNodes() {
                return intakePreviousNodes;
            }

            public ImmutableIntArray outtakePreviousNodes() {
                return outtakePreviousNodes;
            }

            public ImmutableIntArray intakeNextNodes() {
                return node.intakeNextNodes;
            }

            public ImmutableIntArray outtakeNextNodes() {
                return node.outtakeNextNodes;
            }

            public ImmutableIntArray allForwardNodes() {
                return allForwardNodes;
            }

            private record PreviousNodesInfo(int nodeIndex, int orderIndex) {}

            /**
             * Finds the previous nodes for a given node index and next nodes extractor function. The previous nodes are sorted based on their order index, which is determined by the position of the given node index in the next nodes array of each potential previous node.
             * @param nodeIndex The index of the node for which to find previous nodes.
             * @param allNodes An array of all nodes.
             * @param nextNodesExtractor A function that extracts the next nodes from a given node.
             * @return An immutable array of previous node indices, sorted by their order index.
             */
            private static ImmutableIntArray findPreviousNodes(
                    int nodeIndex, FuelNode[] allNodes, Function<FuelNode, ImmutableIntArray> nextNodesExtractor) {
                List<PreviousNodesInfo> previousNodes = new ArrayList<>();

                for (int i = 1; i <= allNodes.length; i++) {
                    ImmutableIntArray nextNodes = nextNodesExtractor.apply(allNodes[i - 1]);
                    if (i != nodeIndex && nextNodes.contains(nodeIndex)) {
                        int orderIndex = nextNodes.indexOf(nodeIndex);
                        previousNodes.add(new PreviousNodesInfo(i, orderIndex));
                    }
                }

                // Sort by orderIndex
                previousNodes.sort(Comparator.comparingInt(PreviousNodesInfo::orderIndex));

                ImmutableIntArray.Builder builder = ImmutableIntArray.builder();
                for (PreviousNodesInfo info : previousNodes) {
                    builder.add(info.nodeIndex);
                }

                return builder.build();
            }
        }

        private static final int OUTTAKE_NODE = 10;
        private static final int[] INTAKE_NODES = new int[] {22, 23, 24, 25};
        private static final Random RANDOM = new Random();
        private static final double FUEL_TOUCH_GROUND_HEIGHT = Inches.of(3).in(Meters);
        private static final double SHOOT_BPS = Double.parseDouble(System.getProperty("shootbps", "5.4"));
        private static final double GRAVITY_BPS = 5.4;

        private final FuelNode[] ROBOT_FUEL_NODES;

        private ManagedFuelNode[] fuelNodes;

        private final List<FuelObject> fuelObjects = new ArrayList<>();
        private final List<FuelObject> fuelObjectsToRemove = new ArrayList<>();
        private double lastShootTime = 0;
        private double lastGravityTime = 0;
        private boolean shootingFuel = false;
        private boolean fuelInFeeder = false;

        @SuppressWarnings("java:S109") /* sim-only constant */
        public FuelManager() {
            if (Constants.RobotState.getMode() == Mode.SIM) {
                ROBOT_FUEL_NODES = new FuelNode[] {
                    new FuelNode( // 1
                            new Pose3d(0.042767, -0.162363, 0.360788, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(6, 11, 2),
                            AnimationType.CURVED),
                    new FuelNode( // 2
                            new Pose3d(-0.075730, -0.258604, 0.352414, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(7, 3),
                            AnimationType.CURVED),
                    new FuelNode( // 3
                            new Pose3d(-0.226887, -0.239875, 0.365525, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(7, 12),
                            AnimationType.CURVED),
                    new FuelNode( // 4
                            new Pose3d(-0.134114, 0.260796, 0.349812, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(9, 5),
                            AnimationType.CURVED),
                    new FuelNode( // 5
                            new Pose3d(-0.257154, 0.172525, 0.367297, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(9, 8),
                            AnimationType.CURVED),
                    new FuelNode( // 6
                            new Pose3d(0.004048, -0.081546, 0.231411, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(7),
                            AnimationType.ANGULAR),
                    new FuelNode( // 7
                            new Pose3d(-0.140056, -0.122741, 0.232295, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(8),
                            AnimationType.ANGULAR),
                    new FuelNode( // 8
                            new Pose3d(-0.227986, -0.002869, 0.248412, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(9),
                            AnimationType.ANGULAR),
                    new FuelNode( // 9
                            new Pose3d(-0.142263, 0.120316, 0.233914, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(10),
                            AnimationType.ANGULAR),
                    new FuelNode( // 10
                            new Pose3d(0.008452, 0.129710, 0.230662, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 11
                            new Pose3d(-0.082750, -0.078407, 0.356944, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(7, 6, 13, 12),
                            AnimationType.CURVED),
                    new FuelNode( // 12
                            new Pose3d(-0.234366, -0.088855, 0.369445, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(8, 7, 21, 13),
                            AnimationType.CURVED),
                    new FuelNode( // 13
                            new Pose3d(-0.159984, 0.055657, 0.362712, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(9, 8, 5),
                            AnimationType.CURVED),
                    new FuelNode( // 14
                            new Pose3d(-0.158976, -0.124385, 0.481838, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(12, 11, 19, 45, 3, 2),
                            AnimationType.CURVED),
                    new FuelNode( // 15
                            new Pose3d(0.018942, -0.263321, 0.462280, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(2, 1, 16),
                            AnimationType.CURVED),
                    new FuelNode( // 16
                            new Pose3d(-0.016632, -0.125166, 0.494538, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(11, 1, 14),
                            AnimationType.CURVED),
                    new FuelNode( // 17
                            new Pose3d(-0.151558, -0.264549, 0.480822, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(3, 2, 14),
                            AnimationType.CURVED),
                    new FuelNode( // 18
                            new Pose3d(-0.248525, 0.268963, 0.463804, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(4, 5, 20),
                            AnimationType.CURVED),
                    new FuelNode( // 19
                            new Pose3d(-0.143752, 0.021584, 0.495808, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(13, 11, 21, 14),
                            AnimationType.CURVED),
                    new FuelNode( // 20
                            new Pose3d(-0.163724, 0.167428, 0.466852, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(4, 13, 5, 19),
                            AnimationType.CURVED),
                    new FuelNode( // 21
                            new Pose3d(-0.290578, 0.034287, 0.416560, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(13, 5, 12, 8),
                            AnimationType.CURVED),
                    new FuelNode( // 22
                            new Pose3d(-0.512706, -0.209347, 0.201676, Rotation3d.kZero),
                            ImmutableIntArray.of(29, 30),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 23
                            new Pose3d(-0.513431, -0.058470, 0.201676, Rotation3d.kZero),
                            ImmutableIntArray.of(28, 31),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 24
                            new Pose3d(-0.513799, 0.091721, 0.201676, Rotation3d.kZero),
                            ImmutableIntArray.of(27, 32),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 25
                            new Pose3d(-0.513655, 0.241544, 0.201676, Rotation3d.kZero),
                            ImmutableIntArray.of(26, 33),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 26
                            new Pose3d(-0.388130, 0.233567, 0.319629, Rotation3d.kZero),
                            ImmutableIntArray.of(42, 43, 33, 27),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 27
                            new Pose3d(-0.388274, 0.083744, 0.319629, Rotation3d.kZero),
                            ImmutableIntArray.of(42, 41, 32, 28, 26),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 28
                            new Pose3d(-0.387906, -0.066448, 0.319629, Rotation3d.kZero),
                            ImmutableIntArray.of(41, 38, 29, 27),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 29
                            new Pose3d(-0.387182, -0.217325, 0.319629, Rotation3d.kZero),
                            ImmutableIntArray.of(38, 39, 30, 28),
                            ImmutableIntArray.of(),
                            AnimationType.CURVED),
                    new FuelNode( // 30
                            new Pose3d(-0.525299, -0.219116, 0.379476, Rotation3d.kZero),
                            ImmutableIntArray.of(39, 37, 46, 31),
                            ImmutableIntArray.of(29, 31),
                            AnimationType.CURVED),
                    new FuelNode( // 31
                            new Pose3d(-0.534914, -0.068239, 0.350774, Rotation3d.kZero),
                            ImmutableIntArray.of(41, 36, 32, 30),
                            ImmutableIntArray.of(28, 32, 30),
                            AnimationType.CURVED),
                    new FuelNode( // 32
                            new Pose3d(-0.535282, 0.081953, 0.350774, Rotation3d.kZero),
                            ImmutableIntArray.of(42, 41, 35, 33, 31),
                            ImmutableIntArray.of(27, 33, 31),
                            AnimationType.CURVED),
                    new FuelNode( // 33
                            new Pose3d(-0.535138, 0.231776, 0.350774, Rotation3d.kZero),
                            ImmutableIntArray.of(42, 26, 34, 32),
                            ImmutableIntArray.of(26, 32),
                            AnimationType.CURVED),
                    new FuelNode( // 34
                            new Pose3d(-0.561755, 0.232763, 0.501158, Rotation3d.kZero),
                            ImmutableIntArray.of(42, 35, 43),
                            ImmutableIntArray.of(33, 42, 35),
                            AnimationType.CURVED),
                    new FuelNode( // 35
                            new Pose3d(-0.561899, 0.082940, 0.501158, Rotation3d.kZero),
                            ImmutableIntArray.of(47, 42, 41, 36, 34),
                            ImmutableIntArray.of(32, 42, 41, 47, 36),
                            AnimationType.CURVED),
                    new FuelNode( // 36
                            new Pose3d(-0.561531, -0.067252, 0.501158, Rotation3d.kZero),
                            ImmutableIntArray.of(41, 46, 47, 37, 35),
                            ImmutableIntArray.of(31, 41, 46, 35, 37),
                            AnimationType.CURVED),
                    new FuelNode( // 37
                            new Pose3d(-0.560807, -0.218129, 0.518430, Rotation3d.kZero),
                            ImmutableIntArray.of(46, 39, 36),
                            ImmutableIntArray.of(30, 46, 39, 36),
                            AnimationType.CURVED),
                    new FuelNode( // 38
                            new Pose3d(-0.345920, -0.156077, 0.425632, Rotation3d.kZero),
                            ImmutableIntArray.of(40, 45, 46, 41, 39),
                            ImmutableIntArray.of(28, 12, 41, 3, 29),
                            AnimationType.CURVED),
                    new FuelNode( // 39
                            new Pose3d(-0.418108, -0.267618, 0.486036, Rotation3d.kZero),
                            ImmutableIntArray.of(40, 46, 37, 38),
                            ImmutableIntArray.of(29, 38, 30, 40, 46, 37),
                            AnimationType.CURVED),
                    new FuelNode( // 40
                            new Pose3d(-0.288452, -0.235402, 0.527460, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(3, 38, 17, 14, 39, 45),
                            AnimationType.CURVED),
                    new FuelNode( // 41
                            new Pose3d(-0.420639, -0.025777, 0.438912, Rotation3d.kZero),
                            ImmutableIntArray.of(45, 46, 47, 36, 35),
                            ImmutableIntArray.of(28, 27, 21, 31, 42, 38),
                            AnimationType.CURVED),
                    new FuelNode( // 42
                            new Pose3d(-0.430398, 0.148363, 0.426080, Rotation3d.kZero),
                            ImmutableIntArray.of(44, 47, 43, 35, 34),
                            ImmutableIntArray.of(27, 26, 33, 32, 5, 43),
                            AnimationType.CURVED),
                    new FuelNode( // 43
                            new Pose3d(-0.385058, 0.267890, 0.505462, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(26, 18, 5, 44, 42),
                            AnimationType.CURVED),
                    new FuelNode( // 44
                            new Pose3d(-0.302101, 0.137020, 0.517145, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(5, 21, 42, 18, 20),
                            AnimationType.CURVED),
                    new FuelNode( // 45
                            new Pose3d(-0.295824, -0.056221, 0.526255, Rotation3d.kZero),
                            ImmutableIntArray.of(),
                            ImmutableIntArray.of(21, 38, 12, 41, 14, 19),
                            AnimationType.CURVED),
                    new FuelNode( // 46
                            new Pose3d(-0.437256, -0.139400, 0.517154, Rotation3d.kZero),
                            ImmutableIntArray.of(45, 40, 39, 37, 36),
                            ImmutableIntArray.of(38, 41, 30, 39, 40, 36),
                            AnimationType.CURVED),
                    new FuelNode( // 47
                            new Pose3d(-0.424376, 0.056077, 0.540793, Rotation3d.kZero),
                            ImmutableIntArray.of(44, 45, 35, 36),
                            ImmutableIntArray.of(41, 21, 42, 45, 44, 35),
                            AnimationType.CURVED)
                };
            } else {
                ROBOT_FUEL_NODES = new FuelNode[0];
            }
        }

        public void init() {
            fuelNodes =
                    Arrays.stream(ROBOT_FUEL_NODES).map(ManagedFuelNode::new).toArray(ManagedFuelNode[]::new);
        }

        public boolean isShootingFuel() {
            return shootingFuel;
        }

        public boolean isFuelInFeeder() {
            return fuelInFeeder;
        }

        public int getFuelCount() {
            return fuelObjects.size();
        }

        public static int getNodeCount() {
            return RobotModel.getFuelManager().ROBOT_FUEL_NODES.length;
        }

        public final void clearFuel() {
            fuelObjects.clear();
            for (ManagedFuelNode node : fuelNodes) {
                node.occupyingObject = null;
            }
        }

        public final void fillFuel() {
            clearFuel();
            Collections.addAll(
                    fuelObjects,
                    Arrays.stream(fuelNodes)
                            .map(node -> new FuelObject(node.node.pose, node))
                            .toArray(FuelObject[]::new));
        }

        public final void addFuelAtNode(ManagedFuelNode node) {
            fuelObjects.add(new FuelObject(node.node.pose, node));
        }

        // TODO: should be 8 not 5 - add the other 3 not in spindexer based off real robot
        public final void preloadFuel() {
            clearFuel();
            addFuelAtNode(fuelNodes[5]);
            addFuelAtNode(fuelNodes[6]);
            addFuelAtNode(fuelNodes[7]);
            addFuelAtNode(fuelNodes[8]);
            addFuelAtNode(fuelNodes[9]);
        }

        public Pose3d fieldToRobotRelativePose(Pose3d fieldPose) {
            Transform3d transform = fieldPose.minus(RobotContainer.model.getRobot());
            return new Pose3d(transform.getTranslation(), transform.getRotation());
        }

        public void intakeFuel(Pose3d fieldPose, Function<FuelObject, Boolean> marker) {
            Pose3d pose = fieldToRobotRelativePose(fieldPose);
            ManagedFuelNode closestIntakeNode = getClosestIntakeNode(pose);

            if (closestIntakeNode == null) {
                throw new IllegalStateException("No available intake nodes to intake fuel into");
            }

            intakeFuel(
                    pose,
                    findNextNode(closestIntakeNode, ManagedFuelNode::allForwardNodes)
                            .orElse(closestIntakeNode),
                    marker);
        }

        public ManagedFuelNode getClosestIntakeNode(Pose3d pose) {
            ManagedFuelNode closestNode = null;
            ManagedFuelNode closestNodeOccupied = null;
            double closestDistance = Double.MAX_VALUE;
            double closestDistanceOccupied = Double.MAX_VALUE;

            for (int nodeIndex : INTAKE_NODES) {
                ManagedFuelNode node = fuelNodes[nodeIndex - 1];
                double distance = node.node.pose.getTranslation().getDistance(pose.getTranslation());
                if (distance < closestDistanceOccupied) {
                    closestDistanceOccupied = distance;
                    closestNodeOccupied = node;
                }

                if (!node.isOccupied() && distance < closestDistance) {
                    closestDistance = distance;
                    closestNode = node;
                }
            }

            if (closestNode == null) {
                closestNode = closestNodeOccupied;
            }

            return closestNode;
        }

        public void intakeFuel(Pose3d initialPose, ManagedFuelNode node, Function<FuelObject, Boolean> marker) {
            for (int i = 0; i < 500; i++) { // multiple iterations to ensure fuel leaves the intake
                forwardPropagateFrom(getIndex(node), ManagedFuelNode::allForwardNodes, new ArrayList<>(), true);
                backPropagateFrom(
                        OUTTAKE_NODE,
                        ManagedFuelNode::outtakePreviousNodes,
                        ManagedFuelNode::outtakeNextNodes,
                        new ArrayList<>(),
                        false);
                if (!node.isOccupied()) {
                    break;
                }
            }

            FuelObject fuel = new FuelObject(initialPose, node);
            fuel.animateTo(node.node.pose, 9, null);
            fuel.setMarker(marker);
            fuelObjects.add(fuel);

            for (int i = 0; i < 500; i++) { // multiple iterations to ensure fuel leaves the intake
                forwardPropagateFrom(getIndex(node), ManagedFuelNode::allForwardNodes, new ArrayList<>(), true);
                backPropagateFrom(
                        OUTTAKE_NODE,
                        ManagedFuelNode::outtakePreviousNodes,
                        ManagedFuelNode::outtakeNextNodes,
                        new ArrayList<>(),
                        false);
                if (!node.isOccupied()) {
                    break;
                }
            }
        }

        public int getIndex(ManagedFuelNode node) {
            for (int i = 0; i < fuelNodes.length; i++) {
                if (fuelNodes[i] == node) {
                    return i + 1; // node indices are 1-based
                }
            }
            return -1;
        }

        public void outtakeFuel(FuelObject fuel) {
            if (fuel.currentNode == null) {
                throw new IllegalStateException("Cannot outtake fuel that is not currently in a node");
            }

            ManagedFuelNode node = fuel.currentNode;

            fuel.currentNode.occupyingObject = null;
            fuel.currentNode = null;

            backPropagateFrom(
                    getIndex(node),
                    ManagedFuelNode::outtakePreviousNodes,
                    ManagedFuelNode::outtakeNextNodes,
                    new ArrayList<>(),
                    true);
        }

        public Optional<FuelObject> outtakeFuel() {
            if (fuelNodes[OUTTAKE_NODE - 1].isOccupied()) {
                FuelObject fuel = fuelNodes[OUTTAKE_NODE - 1].occupyingObject;
                outtakeFuel(fuel);
                return Optional.of(fuel);
            } else {
                backPropagateFrom(
                        OUTTAKE_NODE,
                        ManagedFuelNode::outtakePreviousNodes,
                        ManagedFuelNode::outtakeNextNodes,
                        new ArrayList<>(),
                        true);

                return Optional.empty();
            }
        }

        public void ejectFuel(FuelObject fuel) {
            if (fuel.currentNode == null) {
                throw new IllegalStateException("Cannot eject fuel that is not currently in a node");
            }

            ManagedFuelNode node = fuel.currentNode;

            fuel.currentNode.occupyingObject = null;
            fuel.currentNode = null;

            backPropagateFrom(
                    getIndex(node),
                    ManagedFuelNode::intakeNextNodes,
                    ManagedFuelNode::intakePreviousNodes,
                    new ArrayList<>(),
                    true);
        }

        public void remove(FuelObject fuel) {
            fuelObjectsToRemove.add(fuel);
        }

        public Optional<FuelObject> ejectFuel() {
            List<Integer> occupiedIntakeNodes = new ArrayList<>();
            for (int nodeIndex : INTAKE_NODES) {
                if (fuelNodes[nodeIndex - 1].isOccupied()) {
                    occupiedIntakeNodes.add(nodeIndex);
                }
            }

            if (!occupiedIntakeNodes.isEmpty()) {
                FuelObject fuel = fuelNodes[occupiedIntakeNodes.get(RANDOM.nextInt(occupiedIntakeNodes.size())) - 1]
                        .occupyingObject;
                ejectFuel(fuel);
                return Optional.of(fuel);
            } else {
                backPropagateFrom(
                        INTAKE_NODES[RANDOM.nextInt(INTAKE_NODES.length)],
                        ManagedFuelNode::intakeNextNodes,
                        ManagedFuelNode::intakePreviousNodes,
                        new ArrayList<>(),
                        true);

                return Optional.empty();
            }
        }

        public void forwardPropagateFrom(
                int nodeIndex,
                Function<ManagedFuelNode, ImmutableIntArray> nextNodesExtractor,
                Collection<Integer> visitedNodes,
                boolean moveAngular) {
            if (visitedNodes.contains(nodeIndex)) {
                return; // prevent infinite loops in case of cycles in the graph
            }
            visitedNodes.add(nodeIndex);

            List<Integer> nextNodeIndices = reorderNodes(nextNodesExtractor.apply(fuelNodes[nodeIndex - 1]));
            randomizeNodes(nextNodeIndices);

            nextNodeIndices.forEach(nextNodeIndex ->
                    forwardPropagateFrom(nextNodeIndex, nextNodesExtractor, visitedNodes, moveAngular));

            ManagedFuelNode nextNode = fuelNodes[nodeIndex - 1];
            if (nextNode.isOccupied() && (moveAngular || nextNode.node.outtakeAnimation != AnimationType.ANGULAR)) {
                findNextNode(nextNode, nextNodesExtractor).ifPresent(next -> moveFuel(nextNode, next));
            }
        }

        public void backPropagateFrom(
                int nodeIndex,
                Function<ManagedFuelNode, ImmutableIntArray> previousNodesExtractor,
                Function<ManagedFuelNode, ImmutableIntArray> nextNodesExtractor,
                Collection<Integer> visitedNodes,
                boolean moveAngular) {
            if (visitedNodes.contains(nodeIndex)) {
                return; // prevent infinite loops in case of cycles in the graph
            }
            visitedNodes.add(nodeIndex);

            List<Integer> previousNodeIndices = reorderNodes(previousNodesExtractor.apply(fuelNodes[nodeIndex - 1]));
            randomizeNodes(previousNodeIndices);

            previousNodeIndices.forEach(previousNodeIndex -> {
                ManagedFuelNode previousNode = fuelNodes[previousNodeIndex - 1];
                if (previousNode.isOccupied()
                        && (moveAngular || previousNode.node.outtakeAnimation != AnimationType.ANGULAR)) {
                    findNextNode(previousNode, nextNodesExtractor)
                            .ifPresent(nextNode -> moveFuel(previousNode, nextNode));
                }

                backPropagateFrom(
                        previousNodeIndex, previousNodesExtractor, nextNodesExtractor, visitedNodes, moveAngular);
            });
        }

        public List<Integer> reorderNodes(ImmutableIntArray nodesIndices) {
            List<Integer> result = new ArrayList<>();
            List<Integer> curved = new ArrayList<>();

            nodesIndices.forEach(i -> {
                ManagedFuelNode previousNode = fuelNodes[i - 1];
                if (previousNode.node.outtakeAnimation == AnimationType.ANGULAR) {
                    result.add(i);
                } else {
                    curved.add(i);
                }
            });

            result.addAll(curved);
            return result;
        }

        public void randomizeNodes(List<Integer> nodeIndices) {
            // Randomize with bias towards spindexer nodes being first, because they are directly touching the wheel
            nodeIndices.sort((a, b) -> {
                ManagedFuelNode nodeA = fuelNodes[a - 1];
                ManagedFuelNode nodeB = fuelNodes[b - 1];
                if (nodeA.node.outtakeAnimation == AnimationType.ANGULAR) {
                    return Math.random() < 0.9 ? -1 : 1;
                } else if (nodeB.node.outtakeAnimation == AnimationType.ANGULAR) {
                    return Math.random() < 0.9 ? 1 : -1;
                } else {
                    return Math.random() < 0.5 ? -1 : 1;
                }
            });
        }

        public Optional<ManagedFuelNode> findNextNode(
                ManagedFuelNode node, Function<ManagedFuelNode, ImmutableIntArray> nextNodesExtractor) {
            ImmutableIntArray nextNodes = nextNodesExtractor.apply(node);
            List<Integer> nextNodeIndices = reorderNodes(nextNodes);
            randomizeNodes(nextNodeIndices);

            for (int i = 0; i < nextNodeIndices.size(); i++) {
                ManagedFuelNode nextNode = fuelNodes[nextNodeIndices.get(i) - 1];
                if (!nextNode.isOccupied()) {
                    return Optional.of(nextNode);
                }
            }
            return Optional.empty();
        }

        public void toProjectile(
                FuelObject fuel, AbstractDriveTrainSimulation drivetrainSim, Translation3d velocityOverride) {
            remove(fuel);
            Pose3d robotPose = RobotContainer.model.getRobot();

            Pose3d fuelFieldPose = robotPose.plus(new Transform3d(fuel.pose.getTranslation(), fuel.pose.getRotation()));

            Translation3d robotRelativeVelocity =
                    velocityOverride != null ? velocityOverride : fuel.velocity.getTranslation();

            ChassisSpeeds chassisSpeeds = drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
            final Translation3d chassisTranslationalVelocity =
                    new Translation3d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);
            final Translation3d shooterVelocityDueToChassisRotation =
                    fuel.pose.getTranslation().rotateBy(new Rotation3d(0, 0, chassisSpeeds.omegaRadiansPerSecond));
            final Translation3d shooterVelocity =
                    chassisTranslationalVelocity.plus(shooterVelocityDueToChassisRotation);

            final Translation3d velocity =
                    shooterVelocity.plus(robotRelativeVelocity.rotateBy(robotPose.getRotation()));

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new GamePieceProjectile(
                                    RebuiltFuelOnField.REBUILT_FUEL_INFO,
                                    fuelFieldPose.getTranslation().toTranslation2d(),
                                    velocity.toTranslation2d(),
                                    fuelFieldPose.getZ(),
                                    velocity.getZ(),
                                    fuelFieldPose.getRotation())
                            .withTouchGroundHeight(FUEL_TOUCH_GROUND_HEIGHT)
                            .enableBecomesGamePieceOnFieldAfterTouchGround());
        }

        public boolean hasFuelIntersecting(LineSegment lineSegment) {
            return fuelObjects.stream().anyMatch(fuel -> fuel.intersects(lineSegment));
        }

        public void moveFuel(ManagedFuelNode fromNode, ManagedFuelNode toNode) {
            if (fromNode.occupyingObject == null) {
                throw new IllegalStateException("Cannot move fuel from an unoccupied node");
            }

            if (toNode.isOccupied()) {
                throw new IllegalStateException("Cannot move fuel to an occupied node");
            }

            FuelObject fuel = fromNode.occupyingObject;
            fromNode.occupyingObject = null;
            toNode.occupyingObject = fuel;
            fuel.currentNode = toNode;
            fuel.animateTo(toNode.node.pose, 9, null);
        }

        public List<Pose3d> getFuelPoses() {
            Pose3d robotPose = RobotContainer.model.getRobot();

            return Collections.unmodifiableList(fuelObjects.stream()
                    .map(fuelObject -> robotPose.plus(
                            new Transform3d(fuelObject.pose.getTranslation(), fuelObject.pose.getRotation())))
                    .toList());
        }

        public List<Pose3dObject> getFuelObjects() {
            Pose3d robotPose = RobotContainer.model.getRobot();

            return Collections.unmodifiableList(fuelObjects.stream()
                    .map(fuelObject -> new Pose3dObject(
                            robotPose.plus(
                                    new Transform3d(fuelObject.pose.getTranslation(), fuelObject.pose.getRotation())),
                            System.identityHashCode(fuelObject)))
                    .toList());
        }

        public void update() {
            fuelObjects.forEach(fuelObject -> fuelObject.update(0.02));

            double currentTime = Timer.getTimestamp();

            if (currentTime - lastGravityTime >= 1.0 / GRAVITY_BPS) {
                lastGravityTime = currentTime;

                for (int i = 0; i < INTAKE_NODES.length; i++) {
                    if (!RobotContainer.intake.getSimIO().isExtendingHopperSpaceAvailable()) {
                        forwardPropagateFrom(
                                INTAKE_NODES[i], ManagedFuelNode::intakeNextNodes, new ArrayList<>(), true);
                        backPropagateFrom(
                                OUTTAKE_NODE,
                                ManagedFuelNode::outtakePreviousNodes,
                                ManagedFuelNode::outtakeNextNodes,
                                new ArrayList<>(),
                                false);
                    }
                }
            }

            if (RobotContainer.spindexer.getSimIO().isOuttaking()
                    && RobotContainer.feeder.getSimIO().isOuttaking()
                    && currentTime - lastShootTime >= 1.0 / SHOOT_BPS) {
                lastShootTime = currentTime;

                outtakeFuel().ifPresent(fuel -> {
                    fuelInFeeder = true;
                    fuel.rotateAround(
                            () -> new Translation3d(0.046038, -0.006424, 0.317031),
                            () -> new Translation3d(0, -1, 0),
                            0.317031 - 0.230662,
                            () -> Math.PI / 2,
                            () -> RobotContainer.feeder.getVelocityRps() * 2 * Math.PI,
                            () -> {
                                fuelInFeeder = false;
                                Translation3d shooterOrigin = new Translation3d(0.210586, 0.239469, 0.455638);
                                Translation3d shooterBallStart = new Translation3d(0.127000, 0.127000, 0.358781);
                                fuel.animateTo(
                                        new Pose3d(shooterBallStart, Rotation3d.kZero),
                                        RobotContainer.feeder.getVelocityRps()
                                                * Math.PI
                                                * (Constants.Feeder.WHEEL_DIAMETER.in(Meters) + FUEL_DIAMETER_METERS),
                                        () -> {
                                            shootingFuel = true;
                                            fuel.rotateAround(
                                                    () -> shooterOrigin.rotateAround(
                                                            shooterBallStart,
                                                            new Rotation3d(
                                                                    Rotation2d.fromRotations(
                                                                            RobotContainer.turret
                                                                                    .getPositionRotations()))),
                                                    () -> new Translation3d(0, 1, 0)
                                                            .rotateBy(new Rotation3d(Rotation2d.fromRotations(
                                                                    RobotContainer.turret.getPositionRotations()))),
                                                    0.113106,
                                                    () -> Math.PI - RobotContainer.shooter.getHoodAngle() - Math.PI / 4,
                                                    () -> RobotContainer.shooter.getFlywheelVelocityMps()
                                                            / Constants.Shooter.FLYWHEEL_WHEEL_DIAMETER.in(Meter)
                                                            * 2,
                                                    () -> {
                                                        shootingFuel = false;
                                                        toProjectile(
                                                                fuel,
                                                                RobotContainer.drivetrain.getSwerveDriveSimulation(),
                                                                new Translation3d(
                                                                                RobotContainer.shootOrchestrator
                                                                                        .getTarget()
                                                                                        .map(
                                                                                                t -> t.shotCalculator()
                                                                                                        .flywheelToFuelVelocity(
                                                                                                                RobotContainer
                                                                                                                        .shooter
                                                                                                                        .getFlywheelVelocityMps()))
                                                                                        .orElse(0.0),
                                                                                0,
                                                                                0)
                                                                        .rotateBy(
                                                                                new Rotation3d(
                                                                                        0,
                                                                                        -RobotContainer.shooter
                                                                                                        .getHoodAngle()
                                                                                                - Constants.Shooter
                                                                                                        .HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS,
                                                                                        Units.rotationsToRadians(
                                                                                                RobotContainer.turret
                                                                                                        .getPositionRotations()))));
                                                    });
                                        });
                            });
                });
            }

            fuelObjectsToRemove.forEach(fuelObjects::remove);
            fuelObjectsToRemove.clear();

            Logger.recordOutput("RobotModel/FuelCount", getFuelCount());
        }
    }
}
