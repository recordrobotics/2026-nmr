package frc.robot.commands;

import com.google.common.primitives.ImmutableDoubleArray;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.recordrobotics.ruckig.InputParameter3;
import org.recordrobotics.ruckig.OutputParameter3;
import org.recordrobotics.ruckig.Ruckig3;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;
import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.enums.Synchronization;
import org.recordrobotics.ruckig.wpi.RuckigTelemetry;

public class RuckigAlign extends Command {

    private static final double VELOCITY_TOLERANCE_MULTIPLIER = 5.0;
    private static final double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD = 0.1; // meters
    // slightly larger threshold to avoid deceleration oscillation
    private static final double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD_BYPASS = 0.18; // meters
    private static final double VELOCITY_MODE_BYPASS_VELOCITY_ERROR_THRESHOLD = 1.0; // m/s

    private static final double DECELERATION_ACCEL_THRESHOLD = 0.9; // meters/s^2

    private static final double CURRENT_STATE_MULTIPLIER =
            0.5; // scales down the Ruckig current velocity/accel initialization for faster deceleration

    private static final Ruckig3 ruckig = new Ruckig3("RuckigAlign", RobotContainer.ROBOT_PERIODIC);
    private static final InputParameter3 input = new InputParameter3();
    private static final OutputParameter3 output = new OutputParameter3();

    private static final PIDController xPid = new PIDController(3, 0, 0.03);
    private static final PIDController yPid = new PIDController(3, 0, 0.03);
    private static final PIDController rPid = new PIDController(5, 0, 0.04);

    private static AlignMode currentMode = AlignMode.POSITION;

    private static boolean lastAlignSuccessful = false;

    public record RuckigAlignState(KinematicState kinematicState, AlignMode alignMode) {}

    static {
        input.setDurationDiscretization(DurationDiscretization.Discrete);
        input.setDefaultSynchronization(Synchronization.Phase);
        input.setPerDoFSynchronization(
                new Synchronization[] {Synchronization.Phase, Synchronization.Phase, Synchronization.None});
        setPositionModeTolerance();
        rPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final Supplier<KinematicState> currentStateSupplier;
    private final Supplier<RuckigAlignState> targetStateSupplier;
    private final ImmutableDoubleArray maxVelocity;
    private final ImmutableDoubleArray maxAcceleration;
    private final ImmutableDoubleArray maxDeceleration;
    private final ImmutableDoubleArray maxJerk;
    private final ImmutableDoubleArray maxDejerk;
    private final boolean resetTrajectory;
    private final RuckigAlignGroup<?> group;

    private final AutoControlModifier controlModifier;

    private Result result;
    private final DeceleratingState[] isDecelerating;

    private enum DeceleratingState {
        NOT_DECELERATING,
        DECELERATING,
        DONE_DECELERATING
    }

    public RuckigAlign(
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier,
            Supplier<RuckigAlignState> targetStateSupplier,
            ImmutableDoubleArray maxVelocity,
            ImmutableDoubleArray maxAcceleration,
            ImmutableDoubleArray maxDeceleration,
            ImmutableDoubleArray maxJerk,
            ImmutableDoubleArray maxDejerk) {
        this(
                controlModifier,
                currentStateSupplier,
                targetStateSupplier,
                maxVelocity,
                maxAcceleration,
                maxDeceleration,
                maxJerk,
                maxDejerk,
                true,
                null);
    }

    private RuckigAlign(
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier,
            Supplier<RuckigAlignState> targetStateSupplier,
            ImmutableDoubleArray maxVelocity,
            ImmutableDoubleArray maxAcceleration,
            ImmutableDoubleArray maxDeceleration,
            ImmutableDoubleArray maxJerk,
            ImmutableDoubleArray maxDejerk,
            boolean resetTrajectory,
            RuckigAlignGroup<?> group) {
        this.controlModifier = controlModifier;
        this.currentStateSupplier = currentStateSupplier;
        this.targetStateSupplier = targetStateSupplier;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxJerk = maxJerk;
        this.maxDejerk = maxDejerk;
        this.resetTrajectory = resetTrajectory;
        this.group = group;

        isDecelerating = new DeceleratingState[maxVelocity.length()];
        Arrays.fill(isDecelerating, DeceleratingState.NOT_DECELERATING);

        addRequirements(RobotContainer.drivetrain);
    }

    /**
     * Helper to create a FIELD RELATIVE KinematicState from a Pose2d and ROBOT RELATIVE ChassisSpeeds
     * @param pose the state pose
     * @param speeds the state speeds
     * @param accel the state accelerations
     * @return the KinematicState
     */
    public static KinematicState toKinematicState(Pose2d pose, ChassisSpeeds speeds, ChassisSpeeds accel) {
        return new KinematicState(
                processPoseForRuckig(pose), processChassisSpeeds(speeds, pose), processChassisSpeeds(accel, pose));
    }

    private static double[] processPoseForRuckig(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            SimpleMath.closestTarget(
                    input.getCurrentPosition()[2],
                    SimpleMath.normalizeAngle(pose.getRotation().getRadians()))
        };
    }

    private static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    private static double[] processChassisSpeeds(ChassisSpeeds speeds, Pose2d pose) {
        return chassisSpeedsToArray(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation()));
    }

    private static void setTargetState(KinematicState state) {
        double[] targetPosition = state.position();
        targetPosition[2] =
                SimpleMath.closestTarget(input.getCurrentPosition()[2], SimpleMath.normalizeAngle(targetPosition[2]));
        input.setTargetPosition(targetPosition);
        input.setTargetVelocity(state.velocity());
        input.setTargetAcceleration(state.acceleration());
    }

    private void applyAlignState(RuckigAlignState state) {
        setTargetState(state.kinematicState());
        maximizeConstraints(maxVelocity, getProcessedMaxAcceleration());
        input.setMaxJerk(getProcessedMaxJerk());
        if (state.alignMode() == AlignMode.POSITION) {
            setPositionModeTolerance();
        } else {
            setVelocityModeTolerance();
        }
    }

    private ImmutableDoubleArray getProcessedMaxAcceleration() {
        ImmutableDoubleArray.Builder processedMaxAcceleration = ImmutableDoubleArray.builder(maxAcceleration.length());

        for (int i = 0; i < maxAcceleration.length(); i++) {
            processedMaxAcceleration.add(
                    isDecelerating[i] == DeceleratingState.DECELERATING
                            ? maxDeceleration.get(i)
                            : maxAcceleration.get(i));
        }

        return processedMaxAcceleration.build();
    }

    private double[] getProcessedMaxJerk() {
        double[] processedMaxJerk = maxJerk.toArray();

        for (int i = 0; i < maxJerk.length(); i++) {
            if (isDecelerating[i] == DeceleratingState.DECELERATING) {
                processedMaxJerk[i] = maxDejerk.get(i);
            }
        }

        return processedMaxJerk;
    }

    private static void maximizeConstraints(ImmutableDoubleArray maxVelocity, ImmutableDoubleArray maxAcceleration) {
        if (maxVelocity.length() != maxAcceleration.length()) {
            throw new IllegalArgumentException("maxVelocity and maxAcceleration must have same length");
        }

        double[] maxVelocityOutput = maxVelocity.toArray();
        double[] maxAccelerationOutput = maxAcceleration.toArray();

        double[] currentVel = input.getCurrentVelocity();
        double[] currentAcc = input.getCurrentAcceleration();

        for (int i = 0; i < maxVelocity.length(); i++) {
            if (Math.abs(currentVel[i]) > Math.abs(maxVelocity.get(i))) {
                maxVelocityOutput[i] = Math.abs(currentVel[i]);
            }

            if (Math.abs(currentAcc[i]) > Math.abs(maxAcceleration.get(i))) {
                maxAccelerationOutput[i] = Math.abs(currentAcc[i]);
            }
        }

        input.setMaxVelocity(maxVelocityOutput);
        input.setMaxAcceleration(maxAccelerationOutput);
    }

    private static double[] scaleDoubleArray(double[] array, double scale) {
        double[] scaled = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            scaled[i] = array[i] * scale;
        }
        return scaled;
    }

    private void reset() {
        KinematicState state = currentStateSupplier.get();
        input.setCurrentPosition(state.position());
        input.setCurrentVelocity(scaleDoubleArray(state.velocity(), CURRENT_STATE_MULTIPLIER));
        input.setCurrentAcceleration(scaleDoubleArray(state.acceleration(), CURRENT_STATE_MULTIPLIER));
        xPid.reset();
        yPid.reset();
        rPid.reset();
        result = Result.Working;
        Arrays.fill(isDecelerating, DeceleratingState.NOT_DECELERATING);
    }

    /**
     * Tight tolerance for position mode (final full-stop target)
     */
    private static void setPositionModeTolerance() {
        xPid.setTolerance(Constants.Align.TRANSLATIONAL_TOLERANCE, Constants.Align.TRANSLATIONAL_VELOCITY_TOLERANCE);
        yPid.setTolerance(Constants.Align.TRANSLATIONAL_TOLERANCE, Constants.Align.TRANSLATIONAL_VELOCITY_TOLERANCE);
        rPid.setTolerance(Constants.Align.ROTATIONAL_TOLERANCE, Constants.Align.ROTATIONAL_VELOCITY_TOLERANCE);
        currentMode = AlignMode.POSITION;
    }

    /**
     * Increases the tolerance for velocity mode
     * to insure smooth waypoint following (final target is still in position mode)
     */
    private static void setVelocityModeTolerance() {
        xPid.setTolerance(
                Constants.Align.TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        yPid.setTolerance(
                Constants.Align.TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        rPid.setTolerance(
                Constants.Align.ROTATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        currentMode = AlignMode.VELOCITY;
    }

    public static double[] getCurrentNewPosition() {
        return output.getNewPosition();
    }

    @Override
    public void initialize() {
        // assume already decelerated unless resetTrajectory which already resets it
        if (!resetTrajectory) Arrays.fill(isDecelerating, DeceleratingState.DONE_DECELERATING);

        input.setMaxVelocity(maxVelocity.toArray());
        input.setMaxAcceleration(maxAcceleration.toArray());
        input.setMaxJerk(maxJerk.toArray());

        if (resetTrajectory) {
            reset();
        }

        applyAlignState(targetStateSupplier.get());

        setLastAlignSuccessful(false);

        if (group != null) {
            ruckig.publishWaypointsToTelemetry(group.getTelemetryWaypoints());
        } else {
            ruckig.publishWaypointsToTelemetry(new RuckigTelemetry.WaypointData[] {});
        }
    }

    private static double feedforward(double velocity, double acceleration, double jerk) {
        final double kV = 1.0;
        final double kA = 0.1;
        final double kJ = 0.01;
        return kV * velocity + kA * acceleration + kJ * jerk;
    }

    @Override
    public void execute() {
        applyAlignState(targetStateSupplier.get());

        result = ruckig.update(input, output);

        KinematicState currentState = currentStateSupplier.get();

        ruckig.updateTelemetry(
                currentState.position(), currentState.velocity(), Constants.Frame.FRAME_WITH_BUMPER_WIDTH);

        double[] newPosition = output.getNewPosition();
        double[] newVelocity = output.getNewVelocity();
        double[] newAcceleration = output.getNewAcceleration();
        double[] newJerk = output.getNewJerk();

        for (int i = 0; i < isDecelerating.length; i++) {
            if (Math.abs(newAcceleration[i]) > DECELERATION_ACCEL_THRESHOLD
                    && !SimpleMath.signEq(newVelocity[i], newAcceleration[i])) {
                if (isDecelerating[i] == DeceleratingState.NOT_DECELERATING) {
                    isDecelerating[i] = DeceleratingState.DECELERATING;
                }
            } else if (isDecelerating[i] == DeceleratingState.DECELERATING) {
                isDecelerating[i] = DeceleratingState.DONE_DECELERATING;
            }
        }

        Logger.recordOutput("Ruckig/Decelerating", isDecelerating);

        Logger.recordOutput(
                "Ruckig/Target",
                new Pose2d(
                        input.getTargetPosition()[0],
                        input.getTargetPosition()[1],
                        new Rotation2d(input.getTargetPosition()[2])));

        Logger.recordOutput(
                "Ruckig/Setpoint", new Pose2d(newPosition[0], newPosition[1], new Rotation2d(newPosition[2])));

        // Calculate the new velocities using PID and velocity feedforward
        double vx = xPid.calculate(currentState.position()[0], newPosition[0])
                + feedforward(newVelocity[0], newAcceleration[0], newJerk[0]);
        double vy = yPid.calculate(currentState.position()[1], newPosition[1])
                + feedforward(newVelocity[1], newAcceleration[1], newJerk[1]);
        double vr = rPid.calculate(currentState.position()[2], newPosition[2])
                + feedforward(newVelocity[2], newAcceleration[2], newJerk[2]);

        controlModifier.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(vx, vy, vr),
                        Rotation2d.fromRadians(currentState.position()[2])),
                new double[4],
                new double[4]);

        output.passToInput(input);

        double ex = Math.abs(currentState.position()[0] - newPosition[0]);
        double ey = Math.abs(currentState.position()[1] - newPosition[1]);
        double er = Math.abs(SimpleMath.closestTarget(
                        newPosition[2], SimpleMath.normalizeAngle(currentState.position()[2]))
                - newPosition[2]);

        Logger.recordOutput("Ruckig/Errors", new double[] {ex, ey, er});

        // If the error is too large, reset the trajectory to current position for more accurate motion
        if (ex * ex + ey * ey > 0.5 || Math.abs(er) > 1.0) {
            reset();
        }
    }

    @Override
    public boolean isFinished() {
        boolean finished = false;

        if (currentMode == AlignMode.POSITION) {
            finished = result != Result.Working && xPid.atSetpoint() && yPid.atSetpoint() && rPid.atSetpoint();
        } else if (currentMode == AlignMode.VELOCITY) {
            KinematicState currentState = currentStateSupplier.get();
            double distanceToTarget = Math.hypot(
                    currentState.position()[0] - input.getTargetPosition()[0],
                    currentState.position()[1] - input.getTargetPosition()[1]);

            double velocityError = Math.hypot(
                    input.getCurrentVelocity()[0] - input.getTargetVelocity()[0],
                    input.getCurrentVelocity()[1] - input.getTargetVelocity()[1]);

            finished = distanceToTarget
                            < (velocityError >= VELOCITY_MODE_BYPASS_VELOCITY_ERROR_THRESHOLD
                                    ? VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD_BYPASS
                                    : VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD)
                    || result != Result.Working;

            Logger.recordOutput("Ruckig/DistanceToTarget", distanceToTarget);
            Logger.recordOutput("Ruckig/VelocityError", velocityError);
        }

        if (finished) {
            setLastAlignSuccessful(true);
        }
        return finished;
    }

    public static boolean lastAlignSuccessful() {
        return lastAlignSuccessful;
    }

    private static void setLastAlignSuccessful(boolean value) {
        lastAlignSuccessful = value;
    }

    /**
     * Allows sequential RuckigAlign commands to not reset the trajectory state on every initialize.
     * Useful for following waypoints with velocity mode.
     */
    public static class RuckigAlignGroup<T> {

        private record AlignEntry<T>(Supplier<T> initializer, Function<T, RuckigAlignState> state, double timeout) {}

        private record Group(int startIndex, AutoControlModifier controlModifier) {}

        private final List<AlignEntry<T>> states = new ArrayList<>();
        private final List<Group> groups = new ArrayList<>();
        private final ImmutableDoubleArray maxVelocity;
        private final ImmutableDoubleArray maxAcceleration;
        private final ImmutableDoubleArray maxDeceleration;
        private final ImmutableDoubleArray maxJerk;
        private final ImmutableDoubleArray maxDejerk;

        private final AutoControlModifier defaultControlModifier;

        private final Supplier<KinematicState> currentStateSupplier;

        /**
         * Create a RuckigAlignGroup with the given constraints
         * @param maxVelocity the max velocity for each of the 3 dimensions (x, y, rotation)
         * @param maxAcceleration the max acceleration for each of the 3 dimensions (x, y, rotation)
         * @param maxJerk the max jerk for each of the 3 dimensions (x, y, rotation)
         */
        public RuckigAlignGroup(
                AutoControlModifier defaultControlModifier,
                Supplier<KinematicState> currentStateSupplier,
                ImmutableDoubleArray maxVelocity,
                ImmutableDoubleArray maxAcceleration,
                ImmutableDoubleArray maxDeceleration,
                ImmutableDoubleArray maxJerk,
                ImmutableDoubleArray maxDejerk) {
            this.defaultControlModifier = defaultControlModifier;
            this.currentStateSupplier = currentStateSupplier;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
            this.maxJerk = maxJerk;
            this.maxDejerk = maxDejerk;
        }

        /**
         * Start a new group of align states (used for splitting the group command into multiple)
         * @return this (for chaining)
         */
        public RuckigAlignGroup<T> newGroup() {
            return newGroup(defaultControlModifier);
        }

        /**
         * Start a new group of align states (used for splitting the group command into multiple)
         * @param controlModifier the control modifier to use for this group
         * @return this (for chaining)
         */
        public RuckigAlignGroup<T> newGroup(AutoControlModifier controlModifier) {
            groups.add(new Group(states.size(), controlModifier));
            return this;
        }

        /**
         * Add an align state to the group
         * @param initializer Function to initialize any parameters needed for the state
         * @param state Function to get the RuckigAlignState from the parameters
         * @param timeout Timeout for this align state
         * @return this (for chaining)
         */
        public RuckigAlignGroup<T> addAlign(
                Supplier<T> initializer, Function<T, RuckigAlignState> state, double timeout) {
            states.add(new AlignEntry<>(initializer, state, timeout));
            return this;
        }

        public ImmutableDoubleArray getMaxVelocity() {
            return maxVelocity;
        }

        public ImmutableDoubleArray getMaxAcceleration() {
            return maxAcceleration;
        }

        public ImmutableDoubleArray getMaxJerk() {
            return maxJerk;
        }

        public ImmutableDoubleArray getMaxDeceleration() {
            return maxDeceleration;
        }

        public ImmutableDoubleArray getMaxDejerk() {
            return maxDejerk;
        }

        public Supplier<KinematicState> getCurrentStateSupplier() {
            return currentStateSupplier;
        }

        /**
         * Build the RuckigAlign command sequence for all groups
         * @return the command sequence
         */
        public Command build() {
            return build(0, groups.size() - 1);
        }

        /**
         * Build the RuckigAlign command sequence for a specific group
         * @param group the group index
         * @return the command sequence
         */
        public Command build(int group) {
            return build(group, group);
        }

        /**
         * Build the RuckigAlign command sequence for a range of groups
         * @param startGroup the starting group index (inclusive)
         * @param endGroup the ending group index (inclusive)
         * @return the command sequence
         */
        public Command build(int startGroup, int endGroup) {
            if (groups.isEmpty()) {
                return build(new int[] {});
            }

            return build(SimpleMath.range(startGroup, endGroup));
        }

        /**
         * Build the RuckigAlign command sequence for a range of groups
         * @param groupIds the group indices to include
         * @return the command sequence
         */
        public Command build(int[] groupIds) {
            if (states.isEmpty()) return Commands.none();

            List<Command> commands = new ArrayList<>();

            if (groups.isEmpty()) {
                // If no groups were defined, treat all states as a single group
                addStateCommands(defaultControlModifier, 0, states.size(), commands);
            } else {
                for (int group : groupIds) {
                    if (group < 0 || group >= groups.size()) {
                        throw new IllegalArgumentException("Invalid group index: " + group);
                    }

                    int startIndex = groups.get(group).startIndex;
                    int endIndex = (group + 1 < groups.size()) ? groups.get(group + 1).startIndex : states.size();
                    addStateCommands(groups.get(group).controlModifier, startIndex, endIndex, commands);
                }
            }

            return Commands.sequence(commands.toArray(Command[]::new));
        }

        /**
         * Add states from startIndex (inclusive) to endIndex (exclusive) to the commands list
         * @param controlModifier the control modifier to use for these states
         * @param startIndex the starting state index (inclusive)
         * @param endIndex the ending state index (exclusive)
         * @param commands the list to add the commands to
         */
        private void addStateCommands(
                AutoControlModifier controlModifier, int startIndex, int endIndex, List<Command> commands) {
            for (int i = startIndex; i < endIndex; i++) {
                final int index = i;
                final AlignEntry<T> entry = states.get(index);
                commands.add(Commands.defer(
                                () -> {
                                    final T param = entry.initializer().get();
                                    return new RuckigAlign(
                                            controlModifier,
                                            currentStateSupplier,
                                            () -> entry.state().apply(param),
                                            maxVelocity,
                                            maxAcceleration,
                                            maxDeceleration,
                                            maxJerk,
                                            maxDejerk,
                                            index == 0,
                                            this);
                                },
                                Set.of(RobotContainer.drivetrain))
                        .withTimeout(entry.timeout()));
            }
        }

        private RuckigTelemetry.WaypointData[] getTelemetryWaypoints() {
            RuckigTelemetry.WaypointData[] waypoints = new RuckigTelemetry.WaypointData[states.size()];
            for (int i = 0; i < states.size(); i++) {
                KinematicState ks = states.get(i)
                        .state()
                        .apply(states.get(i).initializer().get())
                        .kinematicState();
                waypoints[i] = new RuckigTelemetry.WaypointData(ks.position(), ks.velocity(), ks.acceleration());
            }
            return waypoints;
        }
    }

    public enum AlignMode {
        POSITION, // Full stop at target
        VELOCITY // Keep moving at target velocity
    }
}
