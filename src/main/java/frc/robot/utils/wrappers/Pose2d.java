package frc.robot.utils.wrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.errorprone.annotations.Immutable;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import java.util.Collection;

@Immutable
@SuppressWarnings({"java:S2176", "Immutable"})
public class Pose2d extends edu.wpi.first.math.geometry.Pose2d {
    /**
     * A preallocated Pose2d representing the origin.
     *
     * <p>This exists to avoid allocations for common poses.
     */
    public static final Pose2d kZero = new Pose2d();

    /** Pose2d protobuf for serialization. */
    public static final edu.wpi.first.math.geometry.proto.Pose2dProto proto =
            new edu.wpi.first.math.geometry.proto.Pose2dProto();

    /** Pose2d struct for serialization. */
    public static final edu.wpi.first.math.geometry.struct.Pose2dStruct struct =
            new edu.wpi.first.math.geometry.struct.Pose2dStruct();

    /** Constructs a pose at the origin facing toward the positive X axis. */
    public Pose2d() {
        super();
    }

    private Pose2d(edu.wpi.first.math.geometry.Pose2d copy) {
        super(copy.getTranslation(), copy.getRotation());
    }

    /**
     * Constructs a pose with the specified translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    @JsonCreator
    public Pose2d(
            @JsonProperty(required = true, value = "translation") edu.wpi.first.math.geometry.Translation2d translation,
            @JsonProperty(required = true, value = "rotation") Rotation2d rotation) {
        super(translation, rotation);
    }

    /**
     * Constructs a pose with x and y translations instead of a separate Translation2d.
     *
     * @param x The x component of the translational component of the pose.
     * @param y The y component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose2d(double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    /**
     * Constructs a pose with x and y translations instead of a separate Translation2d. The X and Y
     * translations will be converted to and tracked as meters.
     *
     * @param x The x component of the translational component of the pose.
     * @param y The y component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose2d(Distance x, Distance y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    /**
     * Constructs a pose with the specified affine transformation matrix.
     *
     * @param matrix The affine transformation matrix.
     * @throws IllegalArgumentException if the affine transformation matrix is invalid.
     */
    public Pose2d(Matrix<N3, N3> matrix) {
        super(matrix);
    }

    /**
     * Transforms the pose by the given transformation and returns the new transformed pose.
     *
     * <pre>
     * [x_new]    [cos, -sin, 0][transform.x]
     * [y_new] += [sin,  cos, 0][transform.y]
     * [t_new]    [  0,    0, 1][transform.t]
     * </pre>
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    @Override
    public Pose2d plus(Transform2d other) {
        return new Pose2d(super.plus(other));
    }

    /**
     * Multiplies the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose2d.
     */
    @Override
    public Pose2d times(double scalar) {
        return new Pose2d(super.times(scalar));
    }

    /**
     * Divides the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose2d.
     */
    @Override
    public Pose2d div(double scalar) {
        return new Pose2d(super.div(scalar));
    }

    /**
     * Rotates the pose around the origin and returns the new pose.
     *
     * @param other The rotation to transform the pose by.
     * @return The transformed pose.
     */
    @Override
    public Pose2d rotateBy(Rotation2d other) {
        return new Pose2d(super.rotateBy(other));
    }

    /**
     * Transforms the pose by the given transformation and returns the new pose. See + operator for
     * the matrix multiplication performed.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    @Override
    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(super.transformBy(other));
    }

    /**
     * Returns the current pose relative to the given pose.
     *
     * <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
     * get the error between the reference and the current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that the current pose will
     *     be converted into.
     * @return The current pose relative to the new origin pose.
     */
    @Override
    public Pose2d relativeTo(edu.wpi.first.math.geometry.Pose2d other) {
        return new Pose2d(super.relativeTo(other));
    }

    /**
     * Rotates the current pose around a point in 2D space.
     *
     * @param point The point in 2D space to rotate around.
     * @param rot The rotation to rotate the pose by.
     * @return The new rotated pose.
     */
    @Override
    public Pose2d rotateAround(edu.wpi.first.math.geometry.Translation2d point, Rotation2d rot) {
        return new Pose2d(super.rotateAround(point, rot));
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
     * Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
     * derivation.
     *
     * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
     * update. When the user runs exp() on the previous known field-relative pose with the argument
     * being the twist, the user will receive the new field-relative pose.
     *
     * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
     * pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
     *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
     *     degrees since the previous pose update, the twist would be Twist2d(0.01, 0.0,
     *     Units.degreesToRadians(0.5)).
     * @return The new pose of the robot.
     */
    @Override
    public Pose2d exp(Twist2d twist) {
        return new Pose2d(super.exp(twist));
    }

    /**
     * Returns the nearest Pose2d from a list of poses. If two or more poses in the list have the same
     * distance from this pose, return the one with the closest rotation component.
     *
     * @param poses The list of poses to find the nearest.
     * @return The nearest Pose2d from the list.
     */
    @Override
    public Pose2d nearest(Collection<edu.wpi.first.math.geometry.Pose2d> poses) {
        return new Pose2d(super.nearest(poses));
    }

    @Override
    public Pose2d interpolate(edu.wpi.first.math.geometry.Pose2d endValue, double t) {
        return new Pose2d(super.interpolate(endValue, t));
    }

    public static Pose2d toImmutable(edu.wpi.first.math.geometry.Pose2d pose) {
        return new Pose2d(pose);
    }
}
