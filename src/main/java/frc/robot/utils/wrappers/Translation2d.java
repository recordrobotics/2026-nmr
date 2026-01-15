package frc.robot.utils.wrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.errorprone.annotations.Immutable;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import java.util.Collection;

@Immutable
@SuppressWarnings("java:S2176")
public class Translation2d extends edu.wpi.first.math.geometry.Translation2d {
    /**
     * A preallocated Translation2d representing the origin.
     *
     * <p>This exists to avoid allocations for common translations.
     */
    public static final Translation2d kZero = new Translation2d();

    /** Translation2d protobuf for serialization. */
    public static final edu.wpi.first.math.geometry.proto.Translation2dProto proto =
            new edu.wpi.first.math.geometry.proto.Translation2dProto();

    /** Translation2d struct for serialization. */
    public static final edu.wpi.first.math.geometry.struct.Translation2dStruct struct =
            new edu.wpi.first.math.geometry.struct.Translation2dStruct();

    /** Constructs a Translation2d with X and Y components equal to zero. */
    public Translation2d() {
        super();
    }

    private Translation2d(edu.wpi.first.math.geometry.Translation2d copy) {
        super(copy.getX(), copy.getY());
    }

    /**
     * Constructs a Translation2d with the X and Y components equal to the provided values.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    @JsonCreator
    public Translation2d(
            @JsonProperty(required = true, value = "x") double x,
            @JsonProperty(required = true, value = "y") double y) {
        super(x, y);
    }

    /**
     * Constructs a Translation2d with the provided distance and angle. This is essentially converting
     * from polar coordinates to Cartesian coordinates.
     *
     * @param distance The distance from the origin to the end of the translation.
     * @param angle The angle between the x-axis and the translation vector.
     */
    public Translation2d(double distance, Rotation2d angle) {
        super(distance, angle);
    }

    /**
     * Constructs a Translation2d with the X and Y components equal to the provided values. The X and
     * Y components will be converted to and tracked as meters.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public Translation2d(Distance x, Distance y) {
        super(x, y);
    }

    /**
     * Constructs a Translation2d from a 2D translation vector. The values are assumed to be in
     * meters.
     *
     * @param vector The translation vector.
     */
    public Translation2d(Vector<N2> vector) {
        super(vector);
    }

    /**
     * Applies a rotation to the translation in 2D space.
     *
     * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
     * angle.
     *
     * <pre>
     * [x_new]   [other.cos, -other.sin][x]
     * [y_new] = [other.sin,  other.cos][y]
     * </pre>
     *
     * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
     * Translation2d of &lt;0, 2&gt;.
     *
     * @param other The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    @Override
    public Translation2d rotateBy(Rotation2d other) {
        return new Translation2d(super.rotateBy(other));
    }

    /**
     * Rotates this translation around another translation in 2D space.
     *
     * <pre>
     * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
     * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
     * </pre>
     *
     * @param other The other translation to rotate around.
     * @param rot The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    @Override
    public Translation2d rotateAround(edu.wpi.first.math.geometry.Translation2d other, Rotation2d rot) {
        return new Translation2d(super.rotateAround(other, rot));
    }

    /**
     * Returns the sum of two translations in 2D space.
     *
     * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    @Override
    public Translation2d plus(edu.wpi.first.math.geometry.Translation2d other) {
        return new Translation2d(super.plus(other));
    }

    /**
     * Returns the difference between two translations.
     *
     * <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
     *
     * @param other The translation to subtract.
     * @return The difference between the two translations.
     */
    @Override
    public Translation2d minus(edu.wpi.first.math.geometry.Translation2d other) {
        return new Translation2d(super.minus(other));
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
     * flipping the point over both axes, or negating all components of the translation.
     *
     * @return The inverse of the current translation.
     */
    @Override
    public Translation2d unaryMinus() {
        return new Translation2d(super.unaryMinus());
    }

    /**
     * Returns the translation multiplied by a scalar.
     *
     * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    @Override
    public Translation2d times(double scalar) {
        return new Translation2d(super.times(scalar));
    }

    /**
     * Returns the translation divided by a scalar.
     *
     * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
     *
     * @param scalar The scalar to multiply by.
     * @return The reference to the new mutated object.
     */
    @Override
    public Translation2d div(double scalar) {
        return new Translation2d(super.div(scalar));
    }

    /**
     * Returns the nearest Translation2d from a list of translations.
     *
     * @param translations The list of translations.
     * @return The nearest Translation2d from the list.
     */
    @Override
    public Translation2d nearest(Collection<edu.wpi.first.math.geometry.Translation2d> translations) {
        return new Translation2d(super.nearest(translations));
    }

    @Override
    public Translation2d interpolate(edu.wpi.first.math.geometry.Translation2d endValue, double t) {
        return new Translation2d(super.interpolate(endValue, t));
    }

    public static Translation2d toImmutable(edu.wpi.first.math.geometry.Translation2d translation) {
        return new Translation2d(translation);
    }
}
