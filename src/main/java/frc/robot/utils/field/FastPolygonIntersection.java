package frc.robot.utils.field;

public final class FastPolygonIntersection {
    private final float[][] polygons;
    private final float[] minX;
    private final float[] maxX;
    private final float[] minY;
    private final float[] maxY;
    private final float[][] edgeX;
    private final float[][] edgeY;

    public FastPolygonIntersection(float[][] polygons) {
        this.polygons = polygons;
        int nPolys = polygons.length;
        minX = new float[nPolys];
        maxX = new float[nPolys];
        minY = new float[nPolys];
        maxY = new float[nPolys];
        edgeX = new float[nPolys][];
        edgeY = new float[nPolys][];

        for (int i = 0; i < nPolys; i++) {
            float[] poly = polygons[i];

            BoundingBox box = computeBoundingBox(poly);
            minX[i] = box.minX;
            maxX[i] = box.maxX;
            minY[i] = box.minY;
            maxY[i] = box.maxY;

            // Precompute edge vectors
            int n = poly.length / 2;
            edgeX[i] = new float[n];
            edgeY[i] = new float[n];
            for (int j = 0; j < n; j++) {
                int next = (j + 1) % n;
                edgeX[i][j] = poly[next * 2] - poly[j * 2];
                edgeY[i][j] = poly[next * 2 + 1] - poly[j * 2 + 1];
            }
        }
    }

    public float[][] getPolygons() {
        return polygons;
    }

    private record BoundingBox(float minX, float maxX, float minY, float maxY) {}

    private static BoundingBox computeBoundingBox(float[] poly) {
        float bMinX = Float.MAX_VALUE;
        float bMaxX = -Float.MAX_VALUE;
        float bMinY = Float.MAX_VALUE;
        float bMaxY = -Float.MAX_VALUE;
        for (int j = 0; j < poly.length; j += 2) {
            float x = poly[j];
            float y = poly[j + 1];
            if (x < bMinX) bMinX = x;
            if (x > bMaxX) bMaxX = x;
            if (y < bMinY) bMinY = y;
            if (y > bMaxY) bMaxY = y;
        }
        return new BoundingBox(bMinX, bMaxX, bMinY, bMaxY);
    }

    /**
     * Test if segment intersects any polygon.
     * @param x1,y1,x2,y2 - segment endpoints
     * @param countInsideAsCollision - if true, segment entirely inside polygon counts as collision
     */
    public boolean intersectsAny(float x1, float y1, float x2, float y2, boolean countInsideAsCollision) {
        float segMinX = Math.min(x1, x2);
        float segMaxX = Math.max(x1, x2);
        float segMinY = Math.min(y1, y2);
        float segMaxY = Math.max(y1, y2);

        for (int i = 0; i < polygons.length; i++) {
            // Quick reject with bounding boxes
            if (segMaxX < minX[i] || segMinX > maxX[i] || segMaxY < minY[i] || segMinY > maxY[i]) {
                continue;
            }

            float[] poly = polygons[i];

            // Narrow phase: check segment edges
            if (segmentIntersectsPolygon(poly, x1, y1, x2, y2)) {
                return true;
            }

            // Optionally check if segment is entirely inside polygon
            if (countInsideAsCollision && pointInConvexPolygon(i, x1, y1) && pointInConvexPolygon(i, x2, y2)) {
                return true;
            }
        }
        return false;
    }

    // Check segment vs one polygon's edges
    private static boolean segmentIntersectsPolygon(float[] poly, float x1, float y1, float x2, float y2) {
        int n = poly.length / 2;
        for (int i = 0; i < n; i++) {
            float x3 = poly[i * 2];
            float y3 = poly[i * 2 + 1];
            float x4 = poly[(i + 1) % n * 2];
            float y4 = poly[(i + 1) % n * 2 + 1];
            if (segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)) {
                return true;
            }
        }
        return false;
    }

    // Optimized point-in-convex-polygon using precomputed edges
    private boolean pointInConvexPolygon(int polyIndex, float px, float py) {
        float[] poly = polygons[polyIndex];
        float[] ex = edgeX[polyIndex];
        float[] ey = edgeY[polyIndex];
        int n = ex.length;

        for (int i = 0; i < n; i++) {
            float pxRel = px - poly[i * 2];
            float pyRel = py - poly[i * 2 + 1];
            if (ex[i] * pyRel - ey[i] * pxRel < 0) { // CCW polygon
                return false; // early exit
            }
        }
        return true;
    }

    // Fast segment intersection test (cross product)
    @SuppressWarnings("java:S1541") // Math repeats for each edge, no cycles
    private static boolean segmentsIntersect(
            float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

        float d1 = direction(x3, y3, x4, y4, x1, y1);
        float d2 = direction(x3, y3, x4, y4, x2, y2);
        float d3 = direction(x1, y1, x2, y2, x3, y3);
        float d4 = direction(x1, y1, x2, y2, x4, y4);

        return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
                || (Math.abs(d1) < 1e-9f && onSegment(x3, y3, x4, y4, x1, y1))
                || (Math.abs(d2) < 1e-9f && onSegment(x3, y3, x4, y4, x2, y2))
                || (Math.abs(d3) < 1e-9f && onSegment(x1, y1, x2, y2, x3, y3))
                || (Math.abs(d4) < 1e-9f && onSegment(x1, y1, x2, y2, x4, y4));
    }

    private static float direction(float xi, float yi, float xj, float yj, float xk, float yk) {
        return (xk - xi) * (yj - yi) - (xj - xi) * (yk - yi);
    }

    private static boolean onSegment(float xi, float yi, float xj, float yj, float xk, float yk) {
        return Math.min(xi, xj) <= xk && xk <= Math.max(xi, xj) && Math.min(yi, yj) <= yk && yk <= Math.max(yi, yj);
    }
}
