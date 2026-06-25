package frc.robot.utils.camera;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;

/**
 * Enum representing different physical camera models with their specifications.
 */
public enum PhysicalCamera {
    /**
     * Limelight 2 Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-2
     */
    LIMELIGHT_2(new PhysicalCameraBuilder()
            .withResolution(1280, 960)
            .withDownscale(2)
            .withFov(74.36)
            .withFps(13)
            .withPxError(0.2, 0.0005)
            .withLatencyMs(35, 5)
            .withStdDevs(0.55, 1.4175, 0.35)),

    /**
     * Limelight 3G Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3g
     */
    LIMELIGHT_3G(new PhysicalCameraBuilder()
            .withResolution(1280, 960)
            .withDownscale(1)
            .withFov(75.8)
            .withFps(14)
            .withPxError(0.2, 0.0005)
            .withLatencyMs(35, 5)
            .withStdDevs(0.55, 1.11574, 0.35)),

    /**
     * Limelight 4 Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4
     */
    LIMELIGHT_4(new PhysicalCameraBuilder()
            .withResolution(1280, 960)
            .withDownscale(1)
            .withFov(91.5)
            .withFps(30)
            .withPxError(0.2, 0.0005)
            .withLatencyMs(35, 5)
            .withStdDevs(1.5, 1.11574, 1.5)
            .withCurrentDraw(Amps.of(1))),

    /**
     * SVPro Global Shutter Camera Specifications
     * https://www.amazon.com/SVPRO-Shutter-Distortion-Free-1920x1200-Computer/dp/B0CC28R5TL
     */
    SVPRO_GLOBAL_SHUTTER(new PhysicalCameraBuilder()
            .withResolution(1920, 1200)
            .withDownscale(1)
            .withFov(83)
            .withFps(23)
            .withPxError(0.2, 0.0005)
            .withLatencyMs(35, 5)
            .withStdDevs(0.55, 1.3, 0.35)),

    /**
     * Arducam 100 FOV Camera Specifications
     * https://www.amazon.com/Arducam-Camera-Computer-Microphone-Windows/dp/B07ZRJDTBQ/?th=1
     */
    ARDUCAM_100_FOV(new PhysicalCameraBuilder()
            .withResolution(1920, 1080)
            .withFov(100)
            .withFps(30)
            .withLatencyMs(35, 5)),

    /**
     * Arducam 160 FOV Camera Specifications
     * https://www.amazon.com/Arducam-Camera-Computer-Microphone-Windows/dp/B07ZS75KZR/?th=1
     */
    ARDUCAM_160_FOV(new PhysicalCameraBuilder()
            .withResolution(1920, 1080)
            .withFov(160)
            .withFps(30)
            .withLatencyMs(35, 5));

    public final int width;
    public final int height;
    public final double downscale;
    public final double fov; // diagonal
    public final double fps;
    public final double pxError;
    public final double pxErrorStdDev;
    public final double latencyMs;
    public final double latencyStdDevMs;

    public final double minStdDevs;
    public final double stdDevsExponent;
    public final double txtyStdDevs;

    public final double aspectRatio;
    public final double horizontalFov;
    public final double verticalFov;

    public final double maxSightRange;
    public final double minTargetAreaPixels;

    public final Current currentDraw;

    PhysicalCamera(PhysicalCameraBuilder builder) {
        this.width = builder.width;
        this.height = builder.height;
        this.downscale = builder.downscale;
        this.fov = builder.fov;
        this.fps = builder.fps;
        this.pxError = builder.pxError;
        this.pxErrorStdDev = builder.pxErrorStdDev;
        this.latencyMs = builder.latencyMs;
        this.latencyStdDevMs = builder.latencyStdDevMs;
        this.minStdDevs = builder.minStdDevs;
        this.stdDevsExponent = builder.stdDevsExponent;
        this.txtyStdDevs = builder.txtyStdDevs;
        this.maxSightRange = builder.maxSightRange;
        this.minTargetAreaPixels = builder.minTargetAreaPixels;

        this.currentDraw = builder.currentDraw;

        this.aspectRatio = (double) getDetectorWidth() / getDetectorHeight();
        double diagonalFovRad = Math.toRadians(fov);
        double halfDiagonal = Math.tan(diagonalFovRad / 2.0);

        double k = 1.0 / Math.sqrt(1 + aspectRatio * aspectRatio);
        double halfHorizontalFov = Math.atan(halfDiagonal * aspectRatio * k);
        double halfVerticalFov = Math.atan(halfDiagonal * k);

        this.horizontalFov = Math.toDegrees(halfHorizontalFov * 2.0);
        this.verticalFov = Math.toDegrees(halfVerticalFov * 2.0);
    }

    /**
     * Gets the width of the camera detector after downscaling.
     * @return The width of the camera detector in pixels.
     */
    public int getDetectorWidth() {
        return (int) Math.floor(width / downscale);
    }

    /**
     * Gets the height of the camera detector after downscaling.
     * @return The height of the camera detector in pixels.
     */
    public int getDetectorHeight() {
        return (int) Math.floor(height / downscale);
    }

    /**
     * Calculates the standard deviations for pose estimates based on average tag distance.
     * @param avgTagDist The average distance to the detected tags.
     * @return The calculated standard deviations.
     */
    public double calculateStdDevs(double avgTagDist) {
        if (avgTagDist <= 0) {
            return minStdDevs;
        }

        return minStdDevs * Math.pow(stdDevsExponent, avgTagDist);
    }

    /**
     * Checks if a point in camera-relative coordinates is within the camera's field of view.
     * @param cameraRelativePosition The position of the point relative to the camera, where +X is forward, +Y is left, and +Z is up.
     * @return True if the point is within the camera's field of view, false otherwise.
     */
    public boolean isPointInFOV(Translation3d cameraRelativePosition) {
        double horizontalAngle = Math.atan2(cameraRelativePosition.getY(), cameraRelativePosition.getX());

        double verticalAngle = Math.atan(cameraRelativePosition.getZ()
                / Math.sqrt(cameraRelativePosition.getX() * cameraRelativePosition.getX()
                        + cameraRelativePosition.getY() * cameraRelativePosition.getY()));

        return Math.abs(horizontalAngle) <= Math.toRadians(horizontalFov / 2.0)
                && Math.abs(verticalAngle) <= Math.toRadians(verticalFov / 2.0);
    }

    public static class PhysicalCameraBuilder {
        private int width;
        private int height;
        private double downscale = 1.0;
        private double fov; // diagonal
        private double fps;
        private double pxError = 0.2;
        private double pxErrorStdDev = 0.0005;
        private double latencyMs = 35;
        private double latencyStdDevMs = 5;
        private double minStdDevs = 0.55;
        private double stdDevsExponent = 1.0;
        private double txtyStdDevs = 0.35;
        private double maxSightRange = 10.0;
        private double minTargetAreaPixels = 24.0;
        private Current currentDraw = Amps.of(4.0 / 12.0);

        public PhysicalCameraBuilder withResolution(int width, int height) {
            this.width = width;
            this.height = height;
            return this;
        }

        public PhysicalCameraBuilder withDownscale(double downscale) {
            this.downscale = downscale;
            return this;
        }

        public PhysicalCameraBuilder withFov(double fov) {
            this.fov = fov;
            return this;
        }

        public PhysicalCameraBuilder withFps(double fps) {
            this.fps = fps;
            return this;
        }

        public PhysicalCameraBuilder withPxError(double pxError, double pxErrorStdDev) {
            this.pxError = pxError;
            this.pxErrorStdDev = pxErrorStdDev;
            return this;
        }

        public PhysicalCameraBuilder withLatencyMs(double latencyMs, double latencyStdDevMs) {
            this.latencyMs = latencyMs;
            this.latencyStdDevMs = latencyStdDevMs;
            return this;
        }

        public PhysicalCameraBuilder withStdDevs(double minStdDevs, double stdDevsExponent, double txtyStdDevs) {
            this.minStdDevs = minStdDevs;
            this.stdDevsExponent = stdDevsExponent;
            this.txtyStdDevs = txtyStdDevs;
            return this;
        }

        public PhysicalCameraBuilder withMaxSightRange(double maxSightRange) {
            this.maxSightRange = maxSightRange;
            return this;
        }

        public PhysicalCameraBuilder withMinTargetAreaPixels(double minTargetAreaPixels) {
            this.minTargetAreaPixels = minTargetAreaPixels;
            return this;
        }

        public PhysicalCameraBuilder withCurrentDraw(Current currentDraw) {
            this.currentDraw = currentDraw;
            return this;
        }
    }
}
