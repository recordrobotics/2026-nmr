package frc.robot.tests.simple;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.field.FieldIntersection;
import frc.robot.utils.field.FieldIntersection.FieldIntersectionOptions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class FieldIntersectionTests {

    private static final FieldIntersection fieldIntersection = new FieldIntersection(FieldIntersectionOptions.DEFAULT);

    @Test
    @DisplayName("Test collision detection with points outside field bounds")
    void testCollidesWithField_OutsideFieldBounds() {
        // Test points outside field boundaries should return true
        Translation2d outsidePoint1 = new Translation2d(-1.0, 4.0);
        Translation2d insidePoint = new Translation2d(5.0, 4.0);

        assertTrue(
                fieldIntersection.collidesWithField(outsidePoint1, insidePoint),
                "Line from outside field should return true");

        Translation2d outsidePoint2 = new Translation2d(20.0, 4.0);
        assertTrue(
                fieldIntersection.collidesWithField(insidePoint, outsidePoint2),
                "Line to outside field should return true");

        assertTrue(
                fieldIntersection.collidesWithField(outsidePoint1, outsidePoint2),
                "Line between outside points should return true");
    }

    @Test
    @DisplayName("Test collision detection with valid field points not hitting obstacles")
    void testCollidesWithField_NoCollision() {
        // Test points in open field areas that shouldn't collide
        Translation2d openPoint1 = new Translation2d(2.0, 3.0);
        Translation2d openPoint2 = new Translation2d(4.0, 7.0);

        assertFalse(
                fieldIntersection.collidesWithField(openPoint1, openPoint2), "Line in open field should not collide");

        // Test same point
        assertFalse(fieldIntersection.collidesWithField(openPoint1, openPoint1), "Same point should not collide");
    }

    @Test
    @DisplayName("Test collision detection at field boundaries")
    void testCollidesWithField_FieldBoundaries() {
        Translation2d justInsideField = new Translation2d(1.0, 1.0);
        Translation2d farInsideField = new Translation2d(5.0, 0.1);

        assertFalse(
                fieldIntersection.collidesWithField(justInsideField, farInsideField),
                "Line between valid field points in open area should not collide");
    }

    @Test
    @DisplayName("Test collision detection with line segments tangent to obstacles")
    void testCollidesWithField_TangentLines() {
        // Test lines that might be tangent to obstacle boundaries
        Translation2d point1 = new Translation2d(1.0, 2.0);
        Translation2d point2 = new Translation2d(3.0, 2.0);

        // This line should be in open field
        assertFalse(fieldIntersection.collidesWithField(point1, point2), "Line in open field should not collide");
    }

    @Test
    @DisplayName("Test collision detection with very short line segments")
    void testCollidesWithField_ShortSegments() {
        // Test very short line segments
        Translation2d point1 = new Translation2d(1.9, 1.25);
        Translation2d point2 = new Translation2d(1.91, 1.25);

        assertFalse(
                fieldIntersection.collidesWithField(point1, point2),
                "Very short line in open field should not collide");

        // Test short segment within an obstacle
        // TODO: fix this test case
        // Translation2d pillarPoint1 = new Translation2d(3.7, 6.6);
        // Translation2d pillarPoint2 = new Translation2d(3.71, 6.6);

        // assertTrue(
        //         fieldIntersection.collidesWithField(pillarPoint1, pillarPoint2),
        //         "Short line within obstacle should collide");
    }

    @Test
    @DisplayName("Test collision detection across field diagonal")
    void testCollidesWithField_FieldDiagonal() {
        // Test long diagonal lines across the field
        Translation2d bottomLeft = new Translation2d(0.5, 0.5);
        Translation2d topRight = new Translation2d(17.0, 7.5);

        assertTrue(
                fieldIntersection.collidesWithField(bottomLeft, topRight),
                "Diagonal line across field should hit obstacles");
    }

    @Test
    @DisplayName("Test collision detection with various open field paths")
    void testCollidesWithField_OpenFieldPaths() {
        // Test several paths that should be clear
        Translation2d[] openPoints = {
            new Translation2d(2.0, 2.0),
            new Translation2d(7.0, 2.0),
            new Translation2d(10.0, 2.0),
            new Translation2d(15.0, 2.0),
            new Translation2d(2.0, 6.0),
            new Translation2d(7.0, 6.0),
            new Translation2d(10.0, 6.0),
            new Translation2d(15.0, 6.0)
        };

        // Test connections between open points
        for (int i = 0; i < openPoints.length - 1; i++) {
            for (int j = i + 1; j < openPoints.length; j++) {
                Translation2d p1 = openPoints[i];
                Translation2d p2 = openPoints[j];
                // Some of these might intersect obstacles, but we test they execute without error
                assertDoesNotThrow(
                        () -> fieldIntersection.collidesWithField(p1, p2),
                        "Collision detection should not throw exceptions");
            }
        }
    }
}
