package frc.robot.utils.camera.positioned.objectdetection;

import edu.wpi.first.math.util.Units;

public enum ObjectDetectionClass {
    BUMPER(Units.inchesToMeters(20.0)),
    FUEL(15.0 / 100.0); // 15 cm

    private final double height;

    ObjectDetectionClass(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
