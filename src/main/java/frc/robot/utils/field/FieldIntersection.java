package frc.robot.utils.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.google.common.collect.MapMaker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.SimpleMath;
import java.util.Arrays;
import java.util.Collections;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public final class FieldIntersection {

    private static final double FIELD_X_MAX = 16.54105;
    private static final double FIELD_Y_MAX = 8.06926;

    private static final double HUB_X_LEN = 1.19380;
    private static final double HUB_Y_LEN = 1.19380;
    private static final double HUB_X = 4.625594;
    private static final double HUB_Y = 4.03463;
    private static final double HUB_RAMP_LENGTH = Inches.of(73.0).in(Meters);

    private static final double UPRIGHT_X_LEN = Inches.of(3.5).in(Meters);
    private static final double UPRIGHT_Y_LEN = Inches.of(1.5).in(Meters);
    private static final double UPRIGHT_OFFSET_FROM_END_WALL = 1.06204;
    private static final double UPRIGHT_OFFSET_FROM_SIDE_WALL = 3.31524;
    private static final double UPRIGHT_Y_SPACING = Inches.of(33.75).in(Meters);

    private static final double TRENCH_WALL_Y_LEN = Inches.of(12.0).in(Meters);
    private static final double TRENCH_WALL_X_LEN = Inches.of(47.0).in(Meters);
    private static final double TRENCH_WALL_OFFSET_FROM_END_WALL = 4.61769;
    private static final double TRENCH_WALL_OFFSET_FROM_SIDE_WALL = 1.43113;

    private static final int MAX_POLYGON_COUNT = 13;

    /**
     * A set of FieldIntersection instances for different configurations.
     * The keys are weakly referenced to allow for garbage collection and concurrency level 1 to have locking write access.
     */
    private static final Set<FieldIntersection> instances = Collections.newSetFromMap(
            new MapMaker().concurrencyLevel(1).weakKeys().<FieldIntersection, Boolean>makeMap());

    private final FastPolygonIntersection intersection;

    private final FieldIntersectionOptions options;

    /**
     * Creates a FieldIntersection object that can be used to check for collisions with field obstacles.
     * @param options The options for configuring the FieldIntersection.
     */
    public FieldIntersection(FieldIntersectionOptions options) {
        this.options = options;

        FastPolygonIntersectionBuilder builder = new FastPolygonIntersectionBuilder(MAX_POLYGON_COUNT)
                // blue tower uprights
                .addRectangle(
                        UPRIGHT_X_LEN,
                        UPRIGHT_Y_LEN,
                        new Pose2d(UPRIGHT_OFFSET_FROM_END_WALL, UPRIGHT_OFFSET_FROM_SIDE_WALL, new Rotation2d()))
                .addRectangle(
                        UPRIGHT_X_LEN,
                        UPRIGHT_Y_LEN,
                        new Pose2d(
                                UPRIGHT_OFFSET_FROM_END_WALL,
                                UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
                                new Rotation2d()))
                // red tower uprights
                .addRectangle(
                        UPRIGHT_X_LEN,
                        UPRIGHT_Y_LEN,
                        new Pose2d(
                                FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
                                FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL,
                                new Rotation2d()))
                .addRectangle(
                        UPRIGHT_X_LEN,
                        UPRIGHT_Y_LEN,
                        new Pose2d(
                                FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
                                FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
                                new Rotation2d()))
                // blue trench wall
                .addRectangle(
                        TRENCH_WALL_X_LEN,
                        TRENCH_WALL_Y_LEN,
                        new Pose2d(
                                TRENCH_WALL_OFFSET_FROM_END_WALL, TRENCH_WALL_OFFSET_FROM_SIDE_WALL, new Rotation2d()))
                .addRectangle(
                        TRENCH_WALL_X_LEN,
                        TRENCH_WALL_Y_LEN,
                        new Pose2d(
                                TRENCH_WALL_OFFSET_FROM_END_WALL,
                                FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                                new Rotation2d()))
                // red trench wall
                .addRectangle(
                        TRENCH_WALL_X_LEN,
                        TRENCH_WALL_Y_LEN,
                        new Pose2d(
                                FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                                TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                                new Rotation2d()))
                .addRectangle(
                        TRENCH_WALL_X_LEN,
                        TRENCH_WALL_Y_LEN,
                        new Pose2d(
                                FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                                FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                                new Rotation2d()));

        if (options.rampCollider() && options.includeTransparentColliders()) {
            // blue hub + ramps
            builder.addRectangle(
                            HUB_X_LEN, HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH, new Pose2d(HUB_X, HUB_Y, new Rotation2d()))
                    // red hub + ramps
                    .addRectangle(
                            HUB_X_LEN,
                            HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH,
                            new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));
        } else if (options.includeTransparentColliders()) {
            // blue hub
            builder.addRectangle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d()))
                    // red hub
                    .addRectangle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));
        } else if (options.rampCollider()) {
            // blue ramp depot
            builder.addRectangle(
                            HUB_X_LEN,
                            HUB_RAMP_LENGTH,
                            new Pose2d(HUB_X, HUB_Y - HUB_Y_LEN / 2.0 - HUB_RAMP_LENGTH / 2.0, new Rotation2d()))
                    // blue ramp outpost
                    .addRectangle(
                            HUB_X_LEN,
                            HUB_RAMP_LENGTH,
                            new Pose2d(HUB_X, HUB_Y + HUB_Y_LEN / 2.0 + HUB_RAMP_LENGTH / 2.0, new Rotation2d()))
                    // red ramp depot
                    .addRectangle(
                            HUB_X_LEN,
                            HUB_RAMP_LENGTH,
                            new Pose2d(
                                    FIELD_X_MAX - HUB_X,
                                    HUB_Y - HUB_Y_LEN / 2.0 - HUB_RAMP_LENGTH / 2.0,
                                    new Rotation2d()))
                    // red ramp outpost
                    .addRectangle(
                            HUB_X_LEN,
                            HUB_RAMP_LENGTH,
                            new Pose2d(
                                    FIELD_X_MAX - HUB_X,
                                    HUB_Y + HUB_Y_LEN / 2.0 + HUB_RAMP_LENGTH / 2.0,
                                    new Rotation2d()));
        } else {
            // blue hub depot
            builder.addPolygon(
                            new Translation2d(HUB_X - HUB_X_LEN / 2, HUB_Y - HUB_Y_LEN / 2),
                            new Translation2d(HUB_X + HUB_X_LEN / 2, HUB_Y - HUB_Y_LEN / 2))
                    // blue hub outpost
                    .addPolygon(
                            new Translation2d(HUB_X - HUB_X_LEN / 2, HUB_Y + HUB_Y_LEN / 2),
                            new Translation2d(HUB_X + HUB_X_LEN / 2, HUB_Y + HUB_Y_LEN / 2))
                    // red hub depot
                    .addPolygon(
                            new Translation2d(FIELD_X_MAX - HUB_X - HUB_X_LEN / 2, HUB_Y - HUB_Y_LEN / 2),
                            new Translation2d(FIELD_X_MAX - HUB_X + HUB_X_LEN / 2, HUB_Y - HUB_Y_LEN / 2))
                    // red hub outpost
                    .addPolygon(
                            new Translation2d(FIELD_X_MAX - HUB_X - HUB_X_LEN / 2, HUB_Y + HUB_Y_LEN / 2),
                            new Translation2d(FIELD_X_MAX - HUB_X + HUB_X_LEN / 2, HUB_Y + HUB_Y_LEN / 2));
        }

        intersection = builder.build();

        instances.add(this);
    }

    /**
     * Check if the line segment between points a and b collides with any field obstacles.
     * @param a  The starting point of the line segment.
     * @param b The ending point of the line segment.
     * @return True if the line segment intersects any field obstacles, false otherwise.
     */
    public boolean collidesWithField(Translation2d a, Translation2d b) {
        if (!SimpleMath.isInField(a) || !SimpleMath.isInField(b)) {
            return true;
        }

        return intersection.intersectsAny((float) a.getX(), (float) a.getY(), (float) b.getX(), (float) b.getY(), true);
    }

    /**
     * Logs the corners of the field polygons for visualization in AdvantageScope.
     */
    public void logPolygons() {
        float[][] polys = intersection.getPolygons();
        Translation2d[][] corners = new Translation2d[polys.length][];
        for (int i = 0; i < polys.length; i++) {
            float[] poly = polys[i];
            corners[i] = new Translation2d[poly.length / 2 + 1];
            for (int j = 0; j <= poly.length; j += 2) {
                corners[i][j / 2] = new Translation2d(poly[j % poly.length], poly[(j + 1) % poly.length]);
            }
        }

        Logger.recordOutput(
                "FieldIntersectionCorners/" + options,
                Arrays.stream(corners)
                        .flatMap(t -> Arrays.stream(t).map(p -> new Pose2d(p, Rotation2d.kZero)))
                        .toArray(Pose2d[]::new));
    }

    public static void logAllInstances() {
        for (FieldIntersection instance : instances) {
            instance.logPolygons();
        }
    }

    public FieldIntersectionOptions getOptions() {
        return options;
    }

    /**
     * Options for configuring the FieldIntersection. This is used to create multiple FieldIntersection objects with different configurations.
     * @param rampCollider Whether to include the bump as colliders. Setting this to false will allowing paths to go through them. This is useful for pathfinding, since the bump can be driven over.
     * @param includeTransparentColliders Whether to include transparent colliders. Transparent colliders are obstacles that can't be passed through, but can be seen through, for example polycarbonate.
     */
    public record FieldIntersectionOptions(boolean rampCollider, boolean includeTransparentColliders) {

        /**
         * The default options include both ramp colliders and transparent colliders.
         */
        public static final FieldIntersectionOptions DEFAULT = new FieldIntersectionOptions(true, true);

        /**
         * Returns a new FieldIntersectionOptions with the given rampCollider value and the same includeTransparentColliders value as this object.
         * @param rampCollider Whether to include the bump as colliders. Setting this to false will allowing paths to go through them. This is useful for pathfinding, since the bump can be driven over.
         * @return A new FieldIntersectionOptions with the given rampCollider value and the same includeTransparentColliders value as this object.
         */
        public FieldIntersectionOptions withRampCollider(boolean rampCollider) {
            return new FieldIntersectionOptions(rampCollider, this.includeTransparentColliders);
        }

        /**
         * Returns a new FieldIntersectionOptions with the given includeTransparentColliders value and the same rampCollider value as this object.
         * @param includeTransparentColliders Whether to include transparent colliders. Transparent colliders are obstacles that can't be passed through, but can be seen through, for example polycarbonate.
         * @return A new FieldIntersectionOptions with the given includeTransparentColliders value and the same rampCollider value as this object.
         */
        public FieldIntersectionOptions withTransparentColliders(boolean includeTransparentColliders) {
            return new FieldIntersectionOptions(this.rampCollider, includeTransparentColliders);
        }

        @Override
        public String toString() {
            return "rampCollider=" + rampCollider + ",includeTransparentColliders=" + includeTransparentColliders;
        }
    }
}
