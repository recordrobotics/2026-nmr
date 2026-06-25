package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.RobotContainer;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.positioned.objectdetection.ObjectDetectionClass;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

public class FieldStateTracker extends ManagedSubsystemBase {

    private static final double OBJECT_TIMEOUT_SECONDS = 5.0;
    private static final double OBJECT_THRESHOLD_SECONDS = 0.002;
    private static final double OBJECT_ASSOCIATION_DISTANCE_THRESHOLD = 0.5; // meters
    private static final double OBJECT_ASSOCIATION_DISTANCE_THRESHOLD_SQUARED =
            OBJECT_ASSOCIATION_DISTANCE_THRESHOLD * OBJECT_ASSOCIATION_DISTANCE_THRESHOLD;
    private static final double NON_ASSOCIATION_COST = 1e6;
    private static final double ASSOCIATION_COST_MARGIN = NON_ASSOCIATION_COST / 2.0;
    private static final double MEASUREMENT_BLEND_WINDOW_SECONDS = 0.01;
    private static final int HUNGARIAN_INITIAL_CAPACITY = 127;
    private static final int HUNGARIAN_GROWTH_FACTOR = 2;
    private static final double MAX_OBJECT_VELOCITY = 4.0;
    private static final double MAX_OBJECT_VELOCITY_SQUARED = MAX_OBJECT_VELOCITY * MAX_OBJECT_VELOCITY;

    /**
     * The cameras used for vision measurements
     */
    private final Set<ObjectDetectionCamera> cameras = Set.of(
            /*
             * Intake Left
             */
            // Cameras.createPhotonVisionObjectDetectionCamera(
            //         Constants.Vision.INTAKE_LEFT_NAME,
            //         PhysicalCamera.ARDUCAM_160_FOV,
            //         Constants.Vision.ROBOT_TO_CAMERA_INTAKE_LEFT,
            //         ImmutableSortedMap.<Integer, ObjectDetectionClass>naturalOrder()
            //                 .put(1, ObjectDetectionClass.BUMPER)
            //                 .put(2, ObjectDetectionClass.FUEL)
            //                 .build()),
            /*
             * Intake Right
             */
            // Cameras.createPhotonVisionObjectDetectionCamera(
            //         Constants.Vision.INTAKE_RIGHT_NAME,
            //         PhysicalCamera.ARDUCAM_160_FOV,
            //         Constants.Vision.ROBOT_TO_CAMERA_INTAKE_RIGHT,
            //         ImmutableSortedMap.<Integer, ObjectDetectionClass>naturalOrder()
            //                 .put(1, ObjectDetectionClass.BUMPER)
            //                 .put(2, ObjectDetectionClass.FUEL)
            //                 .build())
            );

    /**
     * The deferred pose estimations to be added at the end of the calculation phase
     */
    private ConcurrentLinkedQueue<DeferredObjectDetection> deferredObjectDetections = new ConcurrentLinkedQueue<>();

    /**
     * Executor service for asynchronous calculations
     */
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    /**
     * Future for the current calculation task
     */
    private Future<?> calculationFuture = null;

    private Set<FieldObject> fieldObjects = new HashSet<>();
    private final HungarianWorkspace hungarianWorkspace = new HungarianWorkspace();
    private final EnumMap<ObjectDetectionClass, List<FieldObject>> objectsByClassBuffer = createObjectsByClassBuffer();
    private final EnumMap<ObjectDetectionClass, List<DeferredObjectDetection>> detectionsByClassBuffer =
            createDetectionsByClassBuffer();

    private final Field2d field = new Field2d();
    private final SendableBuilder fieldSendableBuilder;

    public static class FieldObject {
        private Pose2d pose;
        private Translation2d velocity = new Translation2d();
        private ObjectDetectionClass detectionClass;
        private double createdTimestamp;
        private double lastSeenTimestamp;

        public FieldObject(Pose2d pose, ObjectDetectionClass detectionClass) {
            this.pose = pose;
            this.detectionClass = detectionClass;
            this.createdTimestamp = Timer.getTimestamp();
            this.lastSeenTimestamp = Timer.getTimestamp();
        }

        public Pose2d getPose() {
            return pose;
        }

        public Pose3d getPose3d() {
            return new Pose3d(
                    pose.getX(), pose.getY(), detectionClass.getHeight() / 2, new Rotation3d(pose.getRotation()));
        }

        public Translation2d getVelocity() {
            return velocity;
        }

        public ObjectDetectionClass getDetectionClass() {
            return detectionClass;
        }

        public boolean isAlive() {
            double currentTime = Timer.getTimestamp();
            return currentTime - lastSeenTimestamp < OBJECT_TIMEOUT_SECONDS
                    && currentTime - createdTimestamp >= OBJECT_THRESHOLD_SECONDS;
        }

        public boolean hasDiedOfAge() {
            double currentTime = Timer.getTimestamp();
            return currentTime - lastSeenTimestamp >= OBJECT_TIMEOUT_SECONDS;
        }

        public boolean isTooYoung() {
            double currentTime = Timer.getTimestamp();
            return currentTime - createdTimestamp < OBJECT_THRESHOLD_SECONDS;
        }

        public void addMeasurement(Pose2d newPose, double confidence, double timestamp) {
            double timeSinceLastSeen = Timer.getTimestamp() - timestamp;
            this.velocity =
                    newPose.getTranslation().minus(this.pose.getTranslation()).div(timeSinceLastSeen);

            if (this.velocity.getSquaredNorm() > MAX_OBJECT_VELOCITY_SQUARED) {
                this.velocity = this.velocity.times(MAX_OBJECT_VELOCITY / this.velocity.getNorm());
            }

            this.pose = this.pose.interpolate(
                    newPose, confidence * Math.min(1.0, MEASUREMENT_BLEND_WINDOW_SECONDS / timeSinceLastSeen));
            this.lastSeenTimestamp = timestamp;
        }

        public void update() {
            double timeSinceLastSeen = Timer.getTimestamp() - lastSeenTimestamp;
            this.pose = new Pose2d(
                    pose.getTranslation().plus(velocity.times(Math.min(0.02, 0.002 / timeSinceLastSeen))),
                    pose.getRotation());
        }
    }

    public FieldStateTracker() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("/SmartDashboard/Autonomous");
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(table.getSubTable("Field"));
        field.initSendable(builder);
        builder.startListeners();
        builder.update();
        fieldSendableBuilder = builder;
    }

    public void addObjectDetectionUpdate(
            Pose2d pose, ObjectDetectionClass detectionClass, double confidence, double timestamp) {
        deferredObjectDetections.add(new DeferredObjectDetection(pose, detectionClass, confidence, timestamp));
    }

    public record DeferredObjectDetection(
            Pose2d pose, ObjectDetectionClass detectionClass, double confidence, double timestampSeconds) {}

    /**
     * Starts the asynchronous calculation phase
     */
    public void startCalculation() {
        deferredObjectDetections.clear();
        calculationFuture = executor.submit(this::performCalculation);
    }

    /**
     * Waits for the end of the calculation and applies all updates
     */
    public void endCalculation() {
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

        logValues();
    }

    private void matchFieldObjectsHungarian() {
        DeferredObjectDetection[] detections = deferredObjectDetections.toArray(new DeferredObjectDetection[0]);
        deferredObjectDetections.clear();
        if (detections.length == 0) {
            return;
        }

        resetClassBuffers();

        for (FieldObject fieldObject : fieldObjects) {
            objectsByClassBuffer.get(fieldObject.getDetectionClass()).add(fieldObject);
        }

        for (DeferredObjectDetection detection : detections) {
            detectionsByClassBuffer.get(detection.detectionClass()).add(detection);
        }

        for (ObjectDetectionClass detectionClass : ObjectDetectionClass.values()) {
            List<DeferredObjectDetection> classDetections = detectionsByClassBuffer.get(detectionClass);
            if (!classDetections.isEmpty()) {
                processMatchesForClass(objectsByClassBuffer.get(detectionClass), classDetections);
            }
        }
    }

    private void resetClassBuffers() {
        objectsByClassBuffer.values().forEach(List::clear);
        detectionsByClassBuffer.values().forEach(List::clear);
    }

    private static EnumMap<ObjectDetectionClass, List<FieldObject>> createObjectsByClassBuffer() {
        EnumMap<ObjectDetectionClass, List<FieldObject>> buffer = new EnumMap<>(ObjectDetectionClass.class);
        for (ObjectDetectionClass detectionClass : ObjectDetectionClass.values()) {
            buffer.put(detectionClass, new ArrayList<>());
        }
        return buffer;
    }

    private static EnumMap<ObjectDetectionClass, List<DeferredObjectDetection>> createDetectionsByClassBuffer() {
        EnumMap<ObjectDetectionClass, List<DeferredObjectDetection>> buffer = new EnumMap<>(ObjectDetectionClass.class);
        for (ObjectDetectionClass detectionClass : ObjectDetectionClass.values()) {
            buffer.put(detectionClass, new ArrayList<>());
        }
        return buffer;
    }

    private void processMatchesForClass(List<FieldObject> classObjects, List<DeferredObjectDetection> classDetections) {
        if (classDetections == null || classDetections.isEmpty()) {
            return;
        }

        if (classObjects == null || classObjects.isEmpty()) {
            createFieldObjectsFromDetections(classDetections);
            return;
        }

        int[] assignments = hungarianWorkspace.assign(classObjects, classDetections);
        boolean[] detectionMatched = applyAssignments(classObjects, classDetections, assignments);
        createFieldObjectsForUnmatchedDetections(classDetections, detectionMatched);
    }

    private void createFieldObjectsFromDetections(List<DeferredObjectDetection> classDetections) {
        for (DeferredObjectDetection detection : classDetections) {
            fieldObjects.add(new FieldObject(detection.pose(), detection.detectionClass()));
        }
    }

    private boolean[] applyAssignments(
            List<FieldObject> classObjects, List<DeferredObjectDetection> classDetections, int[] assignments) {
        boolean[] detectionMatched = new boolean[classDetections.size()];
        for (int i = 0; i < classObjects.size(); i++) {
            int detectionIndex = assignments[i];
            if (hungarianWorkspace.isAssignmentValid(i, detectionIndex)) {
                FieldObject fieldObject = classObjects.get(i);
                DeferredObjectDetection detection = classDetections.get(detectionIndex);
                fieldObject.addMeasurement(detection.pose(), detection.confidence(), detection.timestampSeconds());
                detectionMatched[detectionIndex] = true;
            }
        }
        return detectionMatched;
    }

    private void createFieldObjectsForUnmatchedDetections(
            List<DeferredObjectDetection> classDetections, boolean[] detectionMatched) {
        for (int i = 0; i < classDetections.size(); i++) {
            if (!detectionMatched[i]) {
                DeferredObjectDetection detection = classDetections.get(i);
                fieldObjects.add(new FieldObject(detection.pose(), detection.detectionClass()));
            }
        }
    }

    private static final class HungarianWorkspace {
        private double[][] costMatrix = new double[0][0];
        private double[] potentialRows = new double[0];
        private double[] potentialCols = new double[0];
        private int[] matches = new int[0];
        private int[] predecessors = new int[0];
        private double[] minValues = new double[0];
        private boolean[] usedColumns = new boolean[0];
        private int[] assignmentScratch = new int[0];
        private double[] objectXs = new double[0];
        private double[] objectYs = new double[0];
        private double[] detectionXs = new double[0];
        private double[] detectionYs = new double[0];
        private int capacity = 0;
        private int objectCoordinateCapacity = 0;
        private int detectionCoordinateCapacity = 0;
        private int matrixRows = 0;
        private int matrixColumns = 0;
        private int objectCount = 0;
        private int detectionCount = 0;
        private boolean rowsRepresentObjects = true;

        int[] assign(List<FieldObject> classObjects, List<DeferredObjectDetection> classDetections) {
            objectCount = classObjects.size();
            detectionCount = classDetections.size();
            rowsRepresentObjects = objectCount <= detectionCount;
            matrixRows = rowsRepresentObjects ? objectCount : detectionCount;
            matrixColumns = rowsRepresentObjects ? detectionCount : objectCount;
            int dimension = Math.max(matrixRows, matrixColumns);
            ensureCapacity(dimension);
            ensureCoordinateCapacity(objectCount, detectionCount);
            fillCostMatrix(classObjects, classDetections, dimension);
            runHungarian(dimension);
            Arrays.fill(assignmentScratch, 0, objectCount, -1);
            if (rowsRepresentObjects) {
                populateAssignmentsForObjectRows(dimension);
            } else {
                populateAssignmentsForDetectionRows(dimension);
            }
            return assignmentScratch;
        }

        boolean isAssignmentValid(int objectIndex, int detectionIndex) {
            if (detectionIndex < 0 || detectionIndex >= detectionCount) {
                return false;
            }
            int rowIndex = rowsRepresentObjects ? objectIndex : detectionIndex;
            int columnIndex = rowsRepresentObjects ? detectionIndex : objectIndex;
            if (rowIndex < 0 || rowIndex >= matrixRows || columnIndex < 0 || columnIndex >= matrixColumns) {
                return false;
            }
            return costMatrix[rowIndex][columnIndex] < ASSOCIATION_COST_MARGIN;
        }

        private void ensureCapacity(int requiredDimension) {
            if (requiredDimension <= capacity) {
                return;
            }
            int newCapacity = Math.max(
                    requiredDimension, capacity == 0 ? HUNGARIAN_INITIAL_CAPACITY : capacity * HUNGARIAN_GROWTH_FACTOR);
            costMatrix = new double[newCapacity][newCapacity];
            potentialRows = new double[newCapacity + 1];
            potentialCols = new double[newCapacity + 1];
            matches = new int[newCapacity + 1];
            predecessors = new int[newCapacity + 1];
            minValues = new double[newCapacity + 1];
            usedColumns = new boolean[newCapacity + 1];
            assignmentScratch = new int[newCapacity];
            capacity = newCapacity;
        }

        private void ensureCoordinateCapacity(int requiredObjects, int requiredDetections) {
            if (requiredObjects > objectCoordinateCapacity) {
                int newCapacity = Math.max(
                        requiredObjects,
                        objectCoordinateCapacity == 0
                                ? HUNGARIAN_INITIAL_CAPACITY
                                : objectCoordinateCapacity * HUNGARIAN_GROWTH_FACTOR);
                objectXs = new double[newCapacity];
                objectYs = new double[newCapacity];
                objectCoordinateCapacity = newCapacity;
            }
            if (requiredDetections > detectionCoordinateCapacity) {
                int newCapacity = Math.max(
                        requiredDetections,
                        detectionCoordinateCapacity == 0
                                ? HUNGARIAN_INITIAL_CAPACITY
                                : detectionCoordinateCapacity * HUNGARIAN_GROWTH_FACTOR);
                detectionXs = new double[newCapacity];
                detectionYs = new double[newCapacity];
                detectionCoordinateCapacity = newCapacity;
            }
        }

        private void fillCostMatrix(
                List<FieldObject> classObjects, List<DeferredObjectDetection> classDetections, int dimension) {
            clearActiveRows(dimension);
            cacheObjectCoordinates(classObjects);
            cacheDetectionCoordinates(classDetections);
            if (rowsRepresentObjects) {
                populateCostsForObjectRows();
            } else {
                populateCostsForDetectionRows();
            }
        }

        private void clearActiveRows(int dimension) {
            for (int row = 0; row < matrixRows; row++) {
                Arrays.fill(costMatrix[row], 0, dimension, NON_ASSOCIATION_COST);
            }
        }

        private void cacheObjectCoordinates(List<FieldObject> classObjects) {
            for (int index = 0; index < objectCount; index++) {
                Pose2d pose = classObjects.get(index).getPose();
                objectXs[index] = pose.getX();
                objectYs[index] = pose.getY();
            }
        }

        private void cacheDetectionCoordinates(List<DeferredObjectDetection> classDetections) {
            for (int index = 0; index < detectionCount; index++) {
                Pose2d pose = classDetections.get(index).pose();
                detectionXs[index] = pose.getX();
                detectionYs[index] = pose.getY();
            }
        }

        private void populateCostsForObjectRows() {
            for (int obj = 0; obj < objectCount; obj++) {
                double objectX = objectXs[obj];
                double objectY = objectYs[obj];
                double[] rowCosts = costMatrix[obj];
                for (int det = 0; det < detectionCount; det++) {
                    double cost = squaredDistance(objectX, objectY, detectionXs[det], detectionYs[det]);
                    if (cost <= OBJECT_ASSOCIATION_DISTANCE_THRESHOLD_SQUARED) {
                        rowCosts[det] = cost;
                    }
                }
            }
        }

        private void populateCostsForDetectionRows() {
            for (int det = 0; det < detectionCount; det++) {
                double detectionX = detectionXs[det];
                double detectionY = detectionYs[det];
                double[] rowCosts = costMatrix[det];
                for (int obj = 0; obj < objectCount; obj++) {
                    double cost = squaredDistance(objectXs[obj], objectYs[obj], detectionX, detectionY);
                    if (cost <= OBJECT_ASSOCIATION_DISTANCE_THRESHOLD_SQUARED) {
                        rowCosts[obj] = cost;
                    }
                }
            }
        }

        private void runHungarian(int dimension) {
            Arrays.fill(matches, 0, dimension + 1, 0);
            Arrays.fill(potentialRows, 0, dimension + 1, 0.0);
            Arrays.fill(potentialCols, 0, dimension + 1, 0.0);
            for (int row = 1; row <= matrixRows; row++) {
                augment(row, dimension);
            }
        }

        private void augment(int row, int dimension) {
            matches[0] = row;
            Arrays.fill(minValues, 0, dimension + 1, Double.POSITIVE_INFINITY);
            Arrays.fill(usedColumns, 0, dimension + 1, false);
            int column = 0;
            do {
                usedColumns[column] = true;
                int currentRow = matches[column];
                double delta = Double.POSITIVE_INFINITY;
                int nextColumn = 0;
                for (int candidate = 1; candidate <= dimension; candidate++) {
                    if (!usedColumns[candidate]) {
                        double currentCost = costMatrix[currentRow - 1][candidate - 1]
                                - potentialRows[currentRow]
                                - potentialCols[candidate];
                        if (currentCost < minValues[candidate]) {
                            minValues[candidate] = currentCost;
                            predecessors[candidate] = column;
                        }
                        if (minValues[candidate] < delta) {
                            delta = minValues[candidate];
                            nextColumn = candidate;
                        }
                    }
                }
                updatePotentials(delta, dimension);
                column = nextColumn;
            } while (matches[column] != 0);
            reconstructPath(column);
        }

        private void updatePotentials(double delta, int dimension) {
            for (int column = 0; column <= dimension; column++) {
                if (usedColumns[column]) {
                    potentialRows[matches[column]] += delta;
                    potentialCols[column] -= delta;
                } else {
                    minValues[column] -= delta;
                }
            }
        }

        private void reconstructPath(int column) {
            do {
                int previousColumn = predecessors[column];
                matches[column] = matches[previousColumn];
                column = previousColumn;
            } while (column != 0);
        }

        private void populateAssignmentsForObjectRows(int dimension) {
            for (int column = 1; column <= dimension; column++) {
                int row = matches[column];
                if (row > 0 && row <= matrixRows && column <= detectionCount) {
                    assignmentScratch[row - 1] = column - 1;
                }
            }
        }

        private void populateAssignmentsForDetectionRows(int dimension) {
            for (int column = 1; column <= dimension; column++) {
                int row = matches[column];
                if (row > 0 && row <= matrixRows && column <= objectCount) {
                    assignmentScratch[column - 1] = row - 1;
                }
            }
        }

        private static double squaredDistance(double ax, double ay, double bx, double by) {
            double dx = ax - bx;
            double dy = ay - by;
            return dx * dx + dy * dy;
        }
    }

    /**
     * Logs values to the logger
     */
    private void logValues() {
        field.setRobotPose(RobotContainer.poseSensorFusion.getEstimatedPosition());
        fieldSendableBuilder.update();

        Logger.recordOutput(
                "FieldStateTracker/FieldObjects",
                fieldObjects.stream()
                        .filter(FieldObject::isAlive)
                        .map(FieldObject::getPose3d)
                        .toArray(Pose3d[]::new));

        cameras.stream().forEach(GenericCamera::logValues);
    }

    public void recordMatchReplay() {
        cameras.stream().forEach(GenericCamera::recordMatchReplay);
    }

    public Field2d getField() {
        return field;
    }

    public Set<FieldObject> getFieldObjects() {
        return fieldObjects;
    }

    @Override
    public void periodicManaged() {
        /* logic is asynchronous */
    }

    /**
     * Performs the asynchronous calculation for the pose sensor fusion
     */
    public void performCalculation() {
        cameras.forEach(GenericCamera::periodic);
        cameras.forEach(camera -> {
            camera.addObjectDetectionUpdates(this);
            matchFieldObjectsHungarian();
        });

        fieldObjects.forEach(FieldObject::update);
        fieldObjects.removeIf(FieldObject::hasDiedOfAge);
    }
}
