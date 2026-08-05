// Copyright (c) 2025-2026 KAISER 6989
// https://github.com/haar09/FRC-Rebuilt-BumpSim
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Claude Sonnet 4.6 is used for code generation and refactoring.

package frc.robot.utils.libraries.bumpsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import org.dyn4j.geometry.Vector2;

/** Fuel-bump physics simulation for MapleSim.
 *
 * <p>The sim tracks a single fuel carrier's pose as it climbs the raised bump, then applies a
 * frictionless slide model so it can roll back if it cannot crest the top. The field shape is
 * represented as simple XZ line segments with Y guards.
 */
public class FuelBumpSim {

    // -------------------------------------------------------------------------
    // Field / physics constants
    // -------------------------------------------------------------------------

    /** Control-loop period in seconds. */
    private static final double PERIOD = 0.02;

    /** Gravitational acceleration vector in m/s². */
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81);

    /** Full length of the field in metres. */
    private static final double FIELD_LENGTH = 16.51;

    /** Full width of the field in metres. */
    private static final double FIELD_WIDTH = 8.04;

    /** Start points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_STARTS = {
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    };

    /** End points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_ENDS = {
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    };

    /** First bump segment index. */
    private static final int BUMP_LINE_FIRST = 0;

    /** Last bump segment index. */
    private static final int BUMP_LINE_LAST = BUMP_LINE_STARTS.length - 1;

    // -------------------------------------------------------------------------
    // Tunable physics constants for the fuel carrier
    // -------------------------------------------------------------------------

    /** Effective contact radius against the bump surface in metres. */
    private static final double WHEEL_RADIUS = Constants.Game.FUEL_DIAMETER_METERS / 2.0;

    /** Height offset from the contact surface to the body origin in metres. */
    private static final double CHASSIS_HEIGHT = 0.0;

    /** Coefficient of restitution for vertical bump collisions. */
    private static final double BUMP_COR = 0.15;

    /** Tolerance used to keep endpoint checks stable under floating-point rounding. */
    private static final double SEGMENT_PROJECTION_TOLERANCE = 1e-6;

    /** True while the sim owns the field-X position. */
    private boolean onRamp = false;

    private double zPos = 0.0;

    private double zVel = 0.0;

    /** Absolute field-X position while on the ramp. */
    private double simXPos = 0.0;

    /** Field-X velocity on the ramp in m/s. */
    private double simXVel = 0.0;

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /** Returns true while the sim is handling the ramp slide. */
    public boolean isOnRamp() {
        return onRamp;
    }

    /** Returns the pose MapleSim should use while the sim owns field X. */
    public Pose2d getSimWorldPose(Pose2d latestMaplePose) {
        return new Pose2d(simXPos, latestMaplePose.getY(), latestMaplePose.getRotation());
    }

    /** Advances the simulation by one control period and returns the fuel pose. */
    public Pose3d update(Pose2d fuelPose2d, Vector2 fieldRelativeSpeeds, int subticks, int subTickNum) {
        double vx = fieldRelativeSpeeds.x;
        double dt = PERIOD / subticks;

        double rampAccelXSum = 0.0;
        int contactCount = 0;

        zVel += GRAVITY.getZ() * dt;
        zPos += zVel * dt;

        for (int lineIdx = BUMP_LINE_FIRST; lineIdx <= BUMP_LINE_LAST; lineIdx++) {
            double gax = handleFuelBumpCollision(
                    onRamp ? simXPos : fuelPose2d.getX(), fuelPose2d.getY(), onRamp ? simXVel : vx, lineIdx);
            if (!Double.isNaN(gax)) {
                rampAccelXSum += gax;
                contactCount++;
            }
        }

        if (zPos < 0.0) {
            zPos = 0.0;
            if (zVel < 0.0) zVel = -zVel * BUMP_COR;
        }

        if (contactCount > 0) {
            if (!onRamp) {
                onRamp = true;
                simXPos = fuelPose2d.getX();
                simXVel = vx;
            }
            double avgGravAccelX = (rampAccelXSum / contactCount);
            simXVel += avgGravAccelX * dt;
            simXPos += simXVel * dt;
        } else if (onRamp) {
            boolean allFlat = zPos > 0.01;
            if (allFlat) {
                onRamp = false;
            } else {
                simXPos += simXVel * dt;
            }
        }

        return computePose3d(fuelPose2d);
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** Applies the bump contact response for one fuel contact point. */
    private double handleFuelBumpCollision(double worldX, double worldY, double currentXVel, int lineIdx) {
        Translation3d lineStart = BUMP_LINE_STARTS[lineIdx];
        Translation3d lineEnd = BUMP_LINE_ENDS[lineIdx];

        if (worldY < lineStart.getY() || worldY > lineEnd.getY()) return Double.NaN;

        Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
        Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
        Translation2d pos2d = new Translation2d(worldX, zPos);
        Translation2d lineVec = end2d.minus(start2d);

        Translation2d toFuel = pos2d.minus(start2d);
        double projectionT = toFuel.dot(lineVec) / lineVec.getSquaredNorm();
        Translation2d projected = start2d.plus(lineVec.times(projectionT));

        if (projected.getDistance(start2d) + projected.getDistance(end2d)
                > lineVec.getNorm() + SEGMENT_PROJECTION_TOLERANCE) return Double.NaN;

        double dist = pos2d.getDistance(projected);
        if (dist > WHEEL_RADIUS) return Double.NaN;

        double normalX = -lineVec.getY() / lineVec.getNorm();
        double normalZ = lineVec.getX() / lineVec.getNorm();

        zPos += normalZ * (WHEEL_RADIUS - dist);

        double velDotNormal = currentXVel * normalX + zVel * normalZ;
        if (velDotNormal < 0.0) {
            zVel += normalZ * (-(1.0 + BUMP_COR) * velDotNormal);
        }

        return -GRAVITY.getZ() * normalX * normalZ;
    }

    /** Builds the visible 3D pose from the current 2D pose and bump state. */
    private Pose3d computePose3d(Pose2d fuelPose2d) {
        double visualX = onRamp ? simXPos : fuelPose2d.getX();

        return new Pose3d(
                visualX,
                fuelPose2d.getY(),
                zPos + WHEEL_RADIUS,
                new Rotation3d(0, 0, fuelPose2d.getRotation().getRadians()));
    }
}
