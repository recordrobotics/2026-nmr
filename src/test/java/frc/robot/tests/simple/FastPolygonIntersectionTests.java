package frc.robot.tests.simple;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.utils.field.FastPolygonIntersection;
import java.lang.reflect.*;
import org.junit.jupiter.api.Test;

class FastPolygonIntersectionTests {

    // Helper: simple square polygon CCW
    private static float[] square(float x, float y, float size) {
        return new float[] {x, y, x + size, y, x + size, y + size, x, y + size};
    }

    @Test
    void testGetPolygonsReturnsInput() {
        float[][] polys = {square(0, 0, 1), square(2, 2, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        assertArrayEquals(polys, fpi.getPolygons());
    }

    @Test
    void testIntersectsAny_SegmentOutsidePolygon() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment far away
        assertFalse(fpi.intersectsAny(2, 2, 3, 3, false));
    }

    @Test
    void testIntersectsAny_SegmentIntersectsEdge() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment crosses left edge
        assertTrue(fpi.intersectsAny(-0.5f, 0.5f, 0.5f, 0.5f, false));
    }

    @Test
    void testIntersectsAny_SegmentInsidePolygon_CountInsideTrue() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment entirely inside
        assertTrue(fpi.intersectsAny(0.2f, 0.2f, 0.8f, 0.8f, true));
    }

    @Test
    void testIntersectsAny_SegmentInsidePolygon_CountInsideFalse() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment entirely inside
        assertFalse(fpi.intersectsAny(0.2f, 0.2f, 0.8f, 0.8f, false));
    }

    @Test
    void testIntersectsAny_SegmentTouchesVertex() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment ends at vertex
        assertTrue(fpi.intersectsAny(-1, -1, 0, 0, false));
    }

    @Test
    void testIntersectsAny_SegmentTouchesEdge() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment lies along bottom edge
        assertTrue(fpi.intersectsAny(0, 0, 1, 0, false));
    }

    @Test
    void testIntersectsAny_SegmentOverlapsBoundingBoxButNotPolygon() {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);
        // Segment overlaps bounding box but is outside polygon
        assertFalse(fpi.intersectsAny(-1, 0.5f, -0.1f, 0.5f, false));
    }

    @Test
    void testPointInConvexPolygon_InsideAndOutside() throws Exception {
        float[][] polys = {square(0, 0, 1)};
        FastPolygonIntersection fpi = new FastPolygonIntersection(polys);

        Method pip = FastPolygonIntersection.class.getDeclaredMethod(
                "pointInConvexPolygon", int.class, float.class, float.class);
        pip.setAccessible(true);

        // Inside
        assertTrue((boolean) pip.invoke(fpi, 0, 0.5f, 0.5f));
        // On edge
        assertTrue((boolean) pip.invoke(fpi, 0, 0f, 0.5f));
        // Outside
        assertFalse((boolean) pip.invoke(fpi, 0, 1.5f, 0.5f));
    }

    @Test
    void testSegmentIntersectsPolygon_AllBranches() throws Exception {
        float[] poly = square(0, 0, 1);
        Method sip = FastPolygonIntersection.class.getDeclaredMethod(
                "segmentIntersectsPolygon", float[].class, float.class, float.class, float.class, float.class);
        sip.setAccessible(true);

        // Intersects edge
        assertTrue((boolean) sip.invoke(null, poly, -0.5f, 0.5f, 0.5f, 0.5f));
        // No intersection
        assertFalse((boolean) sip.invoke(null, poly, -1f, -1f, -0.5f, -0.5f));
    }

    @Test
    void testSegmentsIntersect_AllBranches() throws Exception {
        Method si = FastPolygonIntersection.class.getDeclaredMethod(
                "segmentsIntersect",
                float.class,
                float.class,
                float.class,
                float.class,
                float.class,
                float.class,
                float.class,
                float.class);
        si.setAccessible(true);

        // Intersecting segments
        assertTrue((boolean) si.invoke(null, 0f, 0f, 1f, 1f, 0f, 1f, 1f, 0f));
        // Parallel, non-intersecting
        assertFalse((boolean) si.invoke(null, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 1f));
        // Colinear, non-overlapping
        assertFalse((boolean) si.invoke(null, 0f, 0f, 1f, 0f, 2f, 0f, 3f, 0f));
    }

    @Test
    void testDirection() throws Exception {
        Method dir = FastPolygonIntersection.class.getDeclaredMethod(
                "direction", float.class, float.class, float.class, float.class, float.class, float.class);
        dir.setAccessible(true);

        // Negative direction
        assertTrue((float) dir.invoke(null, 0f, 0f, 1f, 0f, 0f, 1f) < 0);
        // Positive direction
        assertTrue((float) dir.invoke(null, 0f, 0f, 1f, 0f, 0f, -1f) > 0);
        // Zero direction (colinear)
        assertEquals(0f, (float) dir.invoke(null, 0f, 0f, 1f, 0f, 2f, 0f));
    }
}
