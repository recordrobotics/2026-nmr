package frc.robot.commands;

import com.google.common.primitives.ImmutableDoubleArray;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RuckigAlign.AlignMode;
import frc.robot.commands.RuckigAlign.RuckigAlignGroup;
import frc.robot.commands.RuckigAlign.RuckigAlignState;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public class WaypointAlign {

    /**
     * Slight multiplier to increase velocity at waypoints and make path smoother
     */
    public static final double WAYPOINT_VELOCITY_MULTIPLIER = 1.05;

    public static List<Pose2d> createWaypointsToTarget(Pose2d target, Transform2d[] targetTransforms) {

        List<Pose2d> waypoints = new ArrayList<>(targetTransforms.length + 1);

        for (Transform2d transform : targetTransforms) {
            waypoints.add(target.plus(transform));
        }
        waypoints.add(target);

        return waypoints;
    }

    private static KinematicState getKinematicStateForWaypoint(
            Pose2d previousWaypoint,
            Pose2d currentWaypoint,
            Pose2d nextWaypoint,
            ImmutableDoubleArray maxVelocity,
            ImmutableDoubleArray maxAcceleration,
            boolean useAcceleration) {

        double px = previousWaypoint.getX();
        double py = previousWaypoint.getY();
        double pr = previousWaypoint.getRotation().getRadians();

        double cx = currentWaypoint.getX();
        double cy = currentWaypoint.getY();
        double cr = SimpleMath.closestTarget(
                pr, SimpleMath.normalizeAngle(currentWaypoint.getRotation().getRadians()));

        double nx = nextWaypoint.getX();
        double ny = nextWaypoint.getY();
        double nr = SimpleMath.closestTarget(
                cr, SimpleMath.normalizeAngle(nextWaypoint.getRotation().getRadians()));

        double dpx = cx - px;
        double dpy = cy - py;
        double dpr = cr - pr;
        double dnx = nx - cx;
        double dny = ny - cy;
        double dnr = nr - cr;

        double lp = Math.hypot(dpx, dpy);
        double ln = Math.hypot(dnx, dny);
        double t = lp < ln ? lp / (2 * ln) : (1 - ln / (2 * lp));

        double[] velocity = calculateVelocity(dpx, dpy, dpr, dnx, dny, dnr, t, maxVelocity);
        double[] acceleration =
                useAcceleration ? calculateAcceleration(dnx, dny, dnr, maxAcceleration) : new double[] {0, 0, 0};

        return new KinematicState(
                new double[] {cx, cy, cr}, // position
                velocity, // velocity
                acceleration // acceleration
                );
    }

    private static double[] calculateVelocity(
            double dpx,
            double dpy,
            double dpr,
            double dnx,
            double dny,
            double dnr,
            double t,
            ImmutableDoubleArray maxVelocity) {
        double dx = dnx * t + dpx * (1 - t);
        double dy = dny * t + dpy * (1 - t);
        double dr = dnr * t + dpr * (1 - t);
        dr = MathUtil.clamp(dr, -maxVelocity.get(2), maxVelocity.get(2));

        double dist = Math.hypot(dx, dy);
        if (dist > 1e-6) {
            double maxV = Math.hypot(maxVelocity.get(0), maxVelocity.get(1));
            if (Math.abs(dr) > 1e-6) {
                maxV /= Math.abs(dr);
            }
            if (dist > maxV) {
                dx = (dx / dist) * maxV;
                dy = (dy / dist) * maxV;
            }
        } else {
            dx = 0.0;
            dy = 0.0;
        }

        dx *= WAYPOINT_VELOCITY_MULTIPLIER;
        dy *= WAYPOINT_VELOCITY_MULTIPLIER;

        return new double[] {dx, dy, dr};
    }

    private static double[] calculateAcceleration(
            double dnx, double dny, double dnr, ImmutableDoubleArray maxAcceleration) {
        dnr = MathUtil.clamp(dnr, -maxAcceleration.get(2), maxAcceleration.get(2));
        double distA = Math.hypot(dnx, dny);

        if (distA > 1e-6) {
            double maxA = Math.hypot(maxAcceleration.get(0), maxAcceleration.get(1));
            if (Math.abs(dnr) > 1e-6) {
                maxA /= Math.abs(dnr);
            }
            if (distA > maxA) {
                dnx = (dnx / distA) * maxA;
                dny = (dny / distA) * maxA;
            }
        } else {
            dnx = 0.0;
            dny = 0.0;
        }

        return new double[] {dnx, dny, dnr};
    }

    private static int getCurrentWaypoint(List<Pose2d> waypoints, KinematicState currentState) {
        double px = currentState.position()[0];
        double py = currentState.position()[1];

        double minDist = Double.MAX_VALUE;
        int closestWaypoint = -1;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            int waypointIndex = i;

            Pose2d wp = waypoints.get(i);
            Pose2d wpn = waypoints.get(i + 1);
            double wx = wp.getX();
            double wy = wp.getY();
            double wnx = wpn.getX();
            double wny = wpn.getY();

            double t = ((px - wx) * (wnx - wx) + (py - wy) * (wny - wy))
                    / ((wnx - wx) * (wnx - wx) + (wny - wy) * (wny - wy));
            if (i == 0 && t < 0) {
                waypointIndex = -1;
            }
            t = MathUtil.clamp(t, 0, 1);
            double cx = wx + t * (wnx - wx);
            double cy = wy + t * (wny - wy);
            double dist = Math.hypot(cx - px, cy - py);
            if (dist < minDist) {
                minDist = dist;
                closestWaypoint = waypointIndex;
            }
        }

        return closestWaypoint;
    }

    private static double moduleToRobotRadians(double moduleValue) {
        return moduleValue / Constants.Swerve.WHEEL_BASE_RADIUS;
    }

    public record KinematicConstraints(
            ImmutableDoubleArray maxVelocity,
            ImmutableDoubleArray maxAcceleration,
            ImmutableDoubleArray maxDeceleration,
            ImmutableDoubleArray maxJerk,
            ImmutableDoubleArray maxDejerk) {
        public static final KinematicConstraints DEFAULT = new KinematicConstraints(
                ImmutableDoubleArray.of(
                        Constants.Align.MAX_VELOCITY / SimpleMath.SQRT2,
                        Constants.Align.MAX_VELOCITY / SimpleMath.SQRT2,
                        Constants.Align.MAX_ANGULAR_VELOCITY), // max velocity
                ImmutableDoubleArray.of(
                        Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2,
                        Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2,
                        moduleToRobotRadians(Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2)), // max acceleration
                ImmutableDoubleArray.of(
                        Constants.Align.MAX_DECELERATION / SimpleMath.SQRT2,
                        Constants.Align.MAX_DECELERATION / SimpleMath.SQRT2,
                        moduleToRobotRadians(Constants.Align.MAX_ACCELERATION
                                / SimpleMath.SQRT2) /* rotation can use accel */), // max deceleration
                ImmutableDoubleArray.of(
                        Constants.Align.MAX_JERK / SimpleMath.SQRT2,
                        Constants.Align.MAX_JERK / SimpleMath.SQRT2,
                        moduleToRobotRadians(Constants.Align.MAX_JERK / SimpleMath.SQRT2)), // max jerk
                ImmutableDoubleArray.of(
                        Constants.Align.MAX_DEJERK / SimpleMath.SQRT2,
                        Constants.Align.MAX_DEJERK / SimpleMath.SQRT2,
                        moduleToRobotRadians(
                                Constants.Align.MAX_JERK / SimpleMath.SQRT2) /* rotation can use jerk */) // max dejerk
                );
    }

    /**
     * Aligns the robot to a target pose. The robot will move to the target pose and stop there.
     * @param target The target pose to align to
     * @param timeout The timeout for the alignment
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot to the target
     */
    public static Command align(
            Pose2d target,
            double timeout,
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier) {
        return align(List.of(target), 0, 0, true, new Double[] {timeout}, controlModifier, currentStateSupplier);
    }

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints
     */
    public static Command align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier) {
        return align(
                waypoints,
                startWaypoint,
                endWaypoint,
                stopAtEndWaypoint,
                waypointTimeouts,
                KinematicConstraints.DEFAULT,
                controlModifier,
                currentStateSupplier);
    }

    private record WaypointData(KinematicState fullStopState, KinematicState velocityState) {}

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param constraints The maximum kinematic constraints to use
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints
     */
    public static Command align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            KinematicConstraints constraints,
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier) {
        RuckigAlignGroup<WaypointData> waypointGroup = new RuckigAlignGroup<>(
                controlModifier,
                currentStateSupplier,
                constraints.maxVelocity,
                constraints.maxAcceleration,
                constraints.maxDeceleration,
                constraints.maxJerk,
                constraints.maxDejerk);
        return align(
                        waypoints,
                        startWaypoint,
                        endWaypoint,
                        stopAtEndWaypoint,
                        waypointTimeouts,
                        waypointGroup,
                        currentStateSupplier)
                .build();
    }

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param waypointGroup The RuckigAlignGroup to use for the alignment (useful for chaining multiple alignments together)
     * @return The RuckigAlignGroup with the waypoints added
     */
    public static RuckigAlignGroup<WaypointData> align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            RuckigAlignGroup<WaypointData> waypointGroup,
            Supplier<KinematicState> currentStateSupplier) {

        validateAlignParameters(waypoints, startWaypoint, endWaypoint, waypointTimeouts);

        for (int i = startWaypoint; i <= endWaypoint; i++) {
            addWaypointToGroup(
                    waypoints,
                    i,
                    endWaypoint,
                    stopAtEndWaypoint,
                    waypointTimeouts,
                    waypointGroup,
                    currentStateSupplier);
        }

        return waypointGroup;
    }

    private static void validateAlignParameters(
            List<Pose2d> waypoints, int startWaypoint, int endWaypoint, Double[] waypointTimeouts) {
        if (waypoints.size() > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        if (startWaypoint < 0 || startWaypoint >= waypoints.size()) {
            throw new IllegalArgumentException("startWaypoint is out of bounds");
        }

        if (endWaypoint < 0 || endWaypoint >= waypoints.size()) {
            throw new IllegalArgumentException("endWaypoint is out of bounds");
        }
    }

    private static void addWaypointToGroup(
            List<Pose2d> waypoints,
            int index,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            RuckigAlignGroup<WaypointData> waypointGroup,
            Supplier<KinematicState> currentStateSupplier) {

        waypointGroup.addAlign(
                () -> createWaypointData(waypoints, index, waypointGroup, currentStateSupplier.get()),
                data -> createRuckigAlignState(data, index, endWaypoint, stopAtEndWaypoint, waypoints.size()),
                waypointTimeouts[index]);
    }

    private static WaypointData createWaypointData(
            List<Pose2d> waypoints,
            int index,
            RuckigAlignGroup<WaypointData> waypointGroup,
            KinematicState currentState) {
        final KinematicState fullStopState = createFullStopState(waypoints.get(index));
        final KinematicState velocityState = createVelocityState(waypoints, index, waypointGroup, currentState);
        return new WaypointData(fullStopState, velocityState);
    }

    private static KinematicState createFullStopState(Pose2d waypoint) {
        return new KinematicState(
                new double[] {
                    waypoint.getX(), waypoint.getY(), waypoint.getRotation().getRadians()
                },
                new double[] {0, 0, 0},
                new double[] {0, 0, 0});
    }

    private static KinematicState createVelocityState(
            List<Pose2d> waypoints,
            int index,
            RuckigAlignGroup<WaypointData> waypointGroup,
            KinematicState currentState) {
        if (index == waypoints.size() - 1) {
            return null;
        }

        Pose2d previousWaypoint = index == 0
                ? new Pose2d(
                        currentState.position()[0],
                        currentState.position()[1],
                        Rotation2d.fromRadians(currentState.position()[2]))
                : waypoints.get(index - 1);

        return getKinematicStateForWaypoint(
                previousWaypoint,
                waypoints.get(index),
                waypoints.get(index + 1),
                waypointGroup.getMaxVelocity(),
                waypointGroup.getMaxAcceleration(),
                true);
    }

    private static RuckigAlignState createRuckigAlignState(
            WaypointData data, int index, int endWaypoint, boolean stopAtEndWaypoint, int waypointsSize) {
        boolean shouldStop = index == waypointsSize - 1 || (stopAtEndWaypoint && index == endWaypoint);
        if (shouldStop) {
            return new RuckigAlignState(data.fullStopState, AlignMode.POSITION);
        } else {
            return new RuckigAlignState(data.velocityState, AlignMode.VELOCITY);
        }
    }

    /**
     * Aligns the robot with a command that runs at a specific waypoint. The command will run
     * immediately when the robot reaches the waypoint specified by {@code commandStartWaypoint} and
     * will wait until the robot reaches the waypoint specified by {@code commandEndWaypoint} before
     * continuing to the next waypoint.
     *
     * @param waypoints The waypoints to align with
     * @param waypointTimeouts The timeouts for each waypoint
     * @param commandStartWaypoint The waypoint when to run the command
     * @param commandEndWaypoint The waypoint before which to wait for command to finish (if start is
     *     -1 and end is 0 then the command will run immediately and wait before entering the segment
     *     between first and second waypoint)
     * @param performCommand The command to run when at the right waypoint
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints and runs the command at the specified waypoint
     */
    public static Command alignWithCommand(
            List<Pose2d> waypoints,
            Double[] waypointTimeouts,
            int commandStartWaypoint,
            int commandEndWaypoint,
            Command performCommand,
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier) {
        return alignWithCommand(
                waypoints,
                waypointTimeouts,
                commandStartWaypoint,
                commandEndWaypoint,
                performCommand,
                KinematicConstraints.DEFAULT,
                controlModifier,
                currentStateSupplier);
    }

    /**
     * Aligns the robot with a command that runs at a specific waypoint. The command will run
     * immediately when the robot reaches the waypoint specified by {@code commandStartWaypoint} and
     * will wait until the robot reaches the waypoint specified by {@code commandEndWaypoint} before
     * continuing to the next waypoint.
     *
     * @param waypoints The waypoints to align with
     * @param waypointTimeouts The timeouts for each waypoint
     * @param commandStartWaypoint The waypoint when to run the command
     * @param commandEndWaypoint The waypoint before which to wait for command to finish (if start is
     *     -1 and end is 0 then the command will run immediately and wait before entering the segment
     *     between first and second waypoint)
     * @param performCommand The command to run when at the right waypoint
     * @param constraints The maximum kinematic constraints to use
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints and runs the command at the specified waypoint
     */
    public static Command alignWithCommand(
            List<Pose2d> waypoints,
            Double[] waypointTimeouts,
            int commandStartWaypoint,
            int commandEndWaypoint,
            Command performCommand,
            KinematicConstraints constraints,
            AutoControlModifier controlModifier,
            Supplier<KinematicState> currentStateSupplier) {

        if (waypoints.size() > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        return Commands.defer(
                () -> {
                    RuckigAlignGroup<WaypointData> waypointGroup = new RuckigAlignGroup<>(
                            controlModifier,
                            currentStateSupplier,
                            constraints.maxVelocity,
                            constraints.maxAcceleration,
                            constraints.maxDeceleration,
                            constraints.maxJerk,
                            constraints.maxDejerk);

                    align(
                            waypoints,
                            Math.min(commandEndWaypoint, getCurrentWaypoint(waypoints, currentStateSupplier.get()) + 1),
                            commandEndWaypoint,
                            false,
                            waypointTimeouts,
                            waypointGroup.newGroup(),
                            currentStateSupplier);
                    align(
                            waypoints,
                            commandEndWaypoint + 1,
                            waypoints.size() - 1,
                            true,
                            waypointTimeouts,
                            waypointGroup.newGroup(),
                            currentStateSupplier);

                    Boolean[] firstAlignFinished = new Boolean[] {false};

                    return waypointGroup
                            .build(0)
                            .andThen(() -> firstAlignFinished[0] = true)
                            /* if waypoint align finished we can assume we are at the end */
                            .alongWith(
                                    new WaitUntilCommand(() -> getCurrentWaypoint(waypoints, currentStateSupplier.get())
                                                            == commandStartWaypoint
                                                    || firstAlignFinished[0])
                                            .andThen(performCommand))
                            .andThen(waypointGroup.build(1));
                },
                Set.of(RobotContainer.drivetrain));
    }
}
