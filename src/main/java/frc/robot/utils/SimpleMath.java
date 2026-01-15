package frc.robot.utils;

import com.google.common.primitives.ImmutableIntArray;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Random;
import java.util.stream.Collectors;

public final class SimpleMath {

    public static final double PI2 = Math.PI * 2;
    public static final double SQRT2 = Math.sqrt(2);
    public static final double SECONDS_PER_MINUTE = 60;

    private static final Random rand = new Random();

    private SimpleMath() {}

    /**
     * Remaps a value between a range to a different range
     *
     * @param value Input value
     * @param inputMin Min range of input value
     * @param inputMax Max range of input value
     * @param outputMin Min range of output value
     * @param outputMax Max range of output value
     * @return Value in range of output range
     * @apiNote THIS DOES NOT CLAMP THE OUTPUT! If input is outside the input range, the output will
     *     also be outside the output range
     */
    public static double remap(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        if (Math.abs(inputMin - inputMax) < 1e-9) {
            throw new IllegalArgumentException("Input range cannot be zero");
        }

        return (value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin;
    }

    /**
     * Takes an input value between -1 and 1 and scales it to the proportion to which it's absolute
     * value is between a minimum threshold and 1 (Function returns 0 if input < threshold) Then
     * multiplies by sensitivity and returns
     *
     * @param input Value between -1 and 1
     * @param threshold Minimum absolute value for input to have an effect (between 0 and 1)
     * @param sensitivity Maximum absolute value of output (between 0 and 1)
     * @return Scaled value between -sensitivity and sensitivity (0 if input
     */
    public static double applyThresholdAndSensitivity(double input, double threshold, double sensitivity) {
        // How much the input is above the threshold (absolute value)
        double subtractThreshold = Math.max(0, Math.abs(input) - threshold);
        // What proportion (threshold to value) is of (threshold to 1)
        double proportion = subtractThreshold / (1 - threshold);
        // Multiplies by spin sensitivity and returns
        return Math.signum(input) * proportion * sensitivity;
    }

    public static boolean isInField(Pose2d pose) {
        return isInField(pose.getTranslation());
    }

    public static boolean isInField(Translation2d pose) {
        return pose.getX() >= 0
                && pose.getY() >= 0
                && pose.getX() <= FlippingUtil.fieldSizeX
                && pose.getY() <= FlippingUtil.fieldSizeY;
    }

    public static double slewRateLimitLinear(double current, double next, double dt, double maxVelocity) {
        if (maxVelocity < 0) {
            Exception e = new IllegalArgumentException();
            MathSharedStore.reportError(
                    "maxVelocity must be a non-negative number, got " + maxVelocity, e.getStackTrace());
            return next;
        }
        double diff = next - current;
        double dist = Math.abs(diff);

        if (dist < 1e-9) {
            return next;
        }
        if (dist > maxVelocity * dt) {
            // Move maximum allowed amount in direction of the difference
            return current + (diff * (maxVelocity * dt / dist));
        }
        return next;
    }

    @SuppressWarnings("java:S109")
    public static Translation2d povToVector(int pov) {
        return switch (pov) {
            case 0 -> new Translation2d(0, 1);
            case 45 -> new Translation2d(1, 1);
            case 90 -> new Translation2d(1, 0);
            case 135 -> new Translation2d(1, -1);
            case 180 -> new Translation2d(0, -1);
            case 225 -> new Translation2d(-1, -1);
            case 270 -> new Translation2d(-1, 0);
            case 315 -> new Translation2d(-1, 1);
            default -> new Translation2d(0, 0);
        };
    }

    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) <= tolerance;
    }

    public static double gaussianNoise(double mean, double stdDev) {
        return rand.nextGaussian(mean, stdDev);
    }

    public static Pose2d poseNoise(Pose2d pose, double stdDev, double stdDevRot) {
        double x = pose.getX() + gaussianNoise(0, stdDev);
        double y = pose.getY() + gaussianNoise(0, stdDev);
        double rot = pose.getRotation().getRadians() + gaussianNoise(0, stdDevRot);

        return new Pose2d(new Translation2d(x, y), new Rotation2d(rot));
    }

    public static Rotation3d gaussianRotation3d(double stdDevX, double stdDevY, double stdDevZ) {
        Vector<N3> noise =
                VecBuilder.fill(gaussianNoise(0, stdDevX), gaussianNoise(0, stdDevY), gaussianNoise(0, stdDevZ));
        return new Rotation3d(noise);
    }

    public static Pose3d poseNoise(Pose3d pose, double stdDev, double stdDevRot) {
        return poseNoise(pose, stdDev, stdDev, stdDev, stdDevRot, stdDevRot, stdDevRot);
    }

    public static Pose3d poseNoise(
            Pose3d pose,
            double stdDevX,
            double stdDevY,
            double stdDevZ,
            double stdDevRotX,
            double stdDevRotY,
            double stdDevRotZ) {
        double x = gaussianNoise(0, stdDevX);
        double y = gaussianNoise(0, stdDevY);
        double z = gaussianNoise(0, stdDevZ);

        return new Pose3d(
                pose.getTranslation().plus(new Translation3d(x, y, z)),
                pose.getRotation().rotateBy(gaussianRotation3d(stdDevRotX, stdDevRotY, stdDevRotZ)));
    }

    public static AprilTagFieldLayout addNoiseToAprilTagFieldLayout(
            AprilTagFieldLayout layout,
            ImmutableIntArray ids,
            double stdDevX,
            double stdDevY,
            double stdDevZ,
            double stdDevRotX,
            double stdDevRotY,
            double stdDevRotZ) {
        Map<Integer, AprilTag> tagMap = layout.getTags().stream().collect(Collectors.toMap(tag -> tag.ID, tag -> tag));

        ids.forEach(id -> {
            AprilTag tag = tagMap.get(id);
            if (tag == null) {
                throw new IllegalArgumentException("Tag ID " + id + " not found in layout");
            }

            Pose3d noisyPose = poseNoise(tag.pose, stdDevX, stdDevY, stdDevZ, stdDevRotX, stdDevRotY, stdDevRotZ);

            tagMap.put(id, new AprilTag(id, noisyPose));
        });

        return new AprilTagFieldLayout(
                new ArrayList<>(tagMap.values()), layout.getFieldLength(), layout.getFieldWidth());
    }

    /**
     * Returns the closest unbounded axis position corresponding to the target angle.
     *
     * @param currentPos Current unbounded axis position (radians)
     * @param targetAngle Target angle (radians, in [-PI, PI])
     * @return Closest unbounded position to the target angle
     */
    public static double closestTarget(double currentPos, double targetAngle) {
        // Compute the nearest multiple of 2pi
        double k = Math.round((currentPos - targetAngle) / PI2);
        // Return the unbounded target position
        return targetAngle + k * PI2;
    }

    /**
     *  Normalize an angle to [-PI, PI] (for use with {@link #closestTarget})
     */
    public static double normalizeAngle(double angle) {
        angle = ((angle + Math.PI) % PI2 + PI2) % PI2 - Math.PI;
        return angle;
    }

    @SuppressWarnings("java:S1244") // comparing signum is fine because it returns EXACT values
    public static boolean signEq(double a, double b) {
        return Math.signum(a) == Math.signum(b);
    }

    /**
     * Returns the average of the given values, or an empty OptionalDouble if no values are given
     * @param values Values to average
     * @return The average of the given values, or an empty OptionalDouble if no values are given
     */
    public static OptionalDouble average(double... values) {
        if (values.length == 0) {
            return OptionalDouble.empty();
        }

        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        return OptionalDouble.of(sum / values.length);
    }

    /**
     * Generates an integer array containing a range of integers from start (inclusive) to end (inclusive)
     * @param start The starting integer (inclusive)
     * @param end The ending integer (inclusive)
     * @return An array of integers from start to end
     */
    public static int[] range(int start, int end) {
        if (end < start) {
            throw new IllegalArgumentException(
                    "End must be greater than or equal to start (" + start + " <= " + end + ")");
        }
        int[] range = new int[end - start + 1];
        for (int i = 0; i < range.length; i++) {
            range[i] = start + i;
        }
        return range;
    }

    /**
     * Interpolates between two Rotation2d with just the radians, allowing for more-than-one-rotation interpolation
     * @param a Starting Rotation2d
     * @param b Ending Rotation2d
     * @param t Interpolation factor [0, 1]
     * @return Interpolated Rotation2d
     */
    public static Rotation2d interpolateRotation2dUnbounded(Rotation2d a, Rotation2d b, double t) {
        double angleA = a.getRadians();
        double angleB = b.getRadians();
        double interpolatedAngle = angleA + (angleB - angleA) * Math.max(0, Math.min(1, t));
        return new Rotation2d(interpolatedAngle);
    }

    public static Pose2d integrateChassisSpeeds(Pose2d currentPose, ChassisSpeeds speeds, double dt) {
        // Rotate into field frame
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, currentPose.getRotation());

        // Field-frame displacement
        double dx = speeds.vxMetersPerSecond * dt;
        double dy = speeds.vyMetersPerSecond * dt;
        double deltatheta = speeds.omegaRadiansPerSecond * dt;

        // New pose
        return new Pose2d(
                currentPose.getX() + dx,
                currentPose.getY() + dy,
                currentPose.getRotation().plus(new Rotation2d(deltatheta)));
    }

    public static Translation2d fromCenterFieldRelativeTranslation2d(Translation2d centerFieldRelative) {
        /*
        Transforms a translation relative to the center of the field to a translation relative to the right corner of the blue alliance wall (origin of the field coordinate system).
        also the center coordinate system is rotated 180 degrees from the field coordinate system, so we also need to flip the x and y coordinates
        */
        return new Translation2d(
                FlippingUtil.fieldSizeX / 2 - centerFieldRelative.getX(),
                FlippingUtil.fieldSizeY / 2 - centerFieldRelative.getY());
    }

    public static Translation3d fromCenterFieldRelativeTranslation3d(Translation3d centerFieldRelative) {
        /*
        Transforms a translation relative to the center of the field to a translation relative to the right corner of the blue alliance wall (origin of the field coordinate system).
        also the center coordinate system is rotated 180 degrees from the field coordinate system, so we also need to flip the x and y coordinates
        */
        return new Translation3d(
                FlippingUtil.fieldSizeX / 2 - centerFieldRelative.getX(),
                FlippingUtil.fieldSizeY / 2 - centerFieldRelative.getY(),
                centerFieldRelative.getZ());
    }

    public static Pose3d withHeight(Pose2d pose, double height) {
        return new Pose3d(new Translation3d(pose.getX(), pose.getY(), height), new Rotation3d(pose.getRotation()));
    }

    public static double rawDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }

    /**
     * Returns true if a is farther from zero than b (in the direction of b).
     * @param a The value to compare
     * @param b The value to compare to (the "direction" of comparison)
     * @return True if a is farther from zero than b (in the direction of b), false otherwise
     */
    public static boolean isAFartherFromZeroThanB(double a, double b) {
        if (b >= 0) {
            return a > b;
        } else {
            return a < b;
        }
    }

    public static Pose3d rotateAroundWithRadius(
            Pose3d pose, Translation3d rotationOrigin, Translation3d axis, double radius, double angleRadians) {

        // Normalize axis
        Translation3d axisUnit = axis.div(axis.getNorm());

        // Vector from origin to pose
        Translation3d relative = pose.getTranslation().minus(rotationOrigin);

        // Project onto axis (parallel component)
        double parallelMag = relative.getX() * axisUnit.getX()
                + relative.getY() * axisUnit.getY()
                + relative.getZ() * axisUnit.getZ();
        Translation3d parallel = axisUnit.times(parallelMag);

        // Perpendicular component (rotation plane vector)
        Translation3d perpendicular = relative.minus(parallel);

        // If perpendicular magnitude is zero, seed a direction
        if (perpendicular.getNorm() < 1e-9) {
            // Choose arbitrary perpendicular direction
            perpendicular = new Translation3d(axisUnit.cross(new Translation3d(1, 0, 0)));
            if (perpendicular.getNorm() < 1e-9) {
                perpendicular = new Translation3d(axisUnit.cross(new Translation3d(0, 1, 0)));
            }
        }

        // Normalize and scale to desired radius
        Translation3d radial = perpendicular.div(perpendicular.getNorm()).times(radius);

        // Build rotation
        Rotation3d axisRotation = new Rotation3d(axisUnit.toVector(), angleRadians);

        // Rotate radial component
        Translation3d rotatedRadial = radial.rotateBy(axisRotation);

        // Final position = origin + parallel + rotatedRadial
        Translation3d finalTranslation = rotationOrigin.plus(parallel).plus(rotatedRadial);

        // Rotate orientation as well
        Rotation3d finalRotation = pose.getRotation().rotateBy(axisRotation);

        return new Pose3d(finalTranslation, finalRotation);
    }
}
