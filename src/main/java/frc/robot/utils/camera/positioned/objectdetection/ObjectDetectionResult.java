package frc.robot.utils.camera.positioned.objectdetection;

public record ObjectDetectionResult(
        int id,
        double confidence,
        double yawDegrees,
        double pitchDegrees,
        double areaPercentage,
        double timestamp,
        TargetCorner corner0,
        TargetCorner corner1,
        TargetCorner corner2,
        TargetCorner corner3) {
    public record TargetCorner(double x, double y) {}
}
