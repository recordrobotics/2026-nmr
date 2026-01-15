package frc.robot.utils.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Builder for FastPolygonIntersection. This class allows you to easily construct a FastPolygonIntersection object by adding polygons and shapes.
 */
public class FastPolygonIntersectionBuilder {

    private final List<float[]> polygons;

    public FastPolygonIntersectionBuilder() {
        polygons = new ArrayList<>();
    }

    /**
     * Creates a FastPolygonIntersectionBuilder with an initial capacity for the given number of polygons. This can improve performance if you know how many polygons you will be adding in advance.
     * @param initialPolygonCapacity the initial capacity for the number of polygons
     */
    public FastPolygonIntersectionBuilder(int initialPolygonCapacity) {
        polygons = new ArrayList<>(initialPolygonCapacity);
    }

    public FastPolygonIntersectionBuilder addPolygons(float[]... polygons) {
        this.polygons.addAll(List.of(polygons));
        return this;
    }

    public FastPolygonIntersectionBuilder addPolygons(Translation2d[]... polygons) {
        for (Translation2d[] poly : polygons) {
            addPolygon(poly);
        }
        return this;
    }

    public FastPolygonIntersectionBuilder addPolygon(Translation2d... polygon) {
        addPolygons(createPolygonFromCorners(polygon));
        return this;
    }

    /**
     * Adds a triangle to the builder. The triangle is defined by its three vertices. This is a convenience method that allows you to easily add triangular colliders without having to create an array of corners.
     * @param a The first vertex of the triangle.
     * @param b The second vertex of the triangle.
     * @param c The third vertex of the triangle.
     * @return The builder instance for chaining.
     */
    public FastPolygonIntersectionBuilder addTriangle(Translation2d a, Translation2d b, Translation2d c) {
        addPolygons(new float[] {
            (float) a.getX(), (float) a.getY(),
            (float) b.getX(), (float) b.getY(),
            (float) c.getX(), (float) c.getY()
        });
        return this;
    }

    /**
     * Adds a rectangle to the builder. The rectangle is defined by its width, height, and pose. The pose specifies the center position and rotation of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * @param pose The pose of the rectangle, specifying its center position and rotation.
     * @return The builder instance for chaining.
     */
    public FastPolygonIntersectionBuilder addRectangle(double width, double height, Pose2d pose) {
        Translation2d halfSize = new Translation2d(width / 2, height / 2);

        addPolygon(
                new Translation2d(-halfSize.getX(), -halfSize.getY())
                        .rotateBy(pose.getRotation())
                        .plus(pose.getTranslation()),
                new Translation2d(halfSize.getX(), -halfSize.getY())
                        .rotateBy(pose.getRotation())
                        .plus(pose.getTranslation()),
                new Translation2d(halfSize.getX(), halfSize.getY())
                        .rotateBy(pose.getRotation())
                        .plus(pose.getTranslation()),
                new Translation2d(-halfSize.getX(), halfSize.getY())
                        .rotateBy(pose.getRotation())
                        .plus(pose.getTranslation()));
        return this;
    }

    /**
     * Converts an array of Translation2d corners into a flat float array representing the polygon vertices. The resulting array will have the format [x1, y1, x2, y2, ..., xn, yn], where (xi, yi) are the coordinates of the i-th corner. This is the format expected by the FastPolygonIntersection class.
     * @param corners The array of Translation2d corners.
     * @return A flat float array representing the polygon vertices.
     */
    public static float[] createPolygonFromCorners(Translation2d[] corners) {
        float[] poly = new float[corners.length * 2];
        for (int i = 0; i < corners.length; i++) {
            poly[i * 2] = (float) corners[i].getX();
            poly[i * 2 + 1] = (float) corners[i].getY();
        }
        return poly;
    }

    public FastPolygonIntersection build() {
        return new FastPolygonIntersection(polygons.toArray(float[][]::new));
    }
}
