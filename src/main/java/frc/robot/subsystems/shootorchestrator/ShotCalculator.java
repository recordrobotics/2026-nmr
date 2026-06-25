package frc.robot.subsystems.shootorchestrator;

public interface ShotCalculator {
    record ShotCalculation(
            double shootAngleRadians,
            double fuelVelocityMagnitudeMps,
            double timeOfFlightSeconds,
            double allowableShootAngleMinRadians,
            double allowableShootAngleMaxRadians,
            double allowableVelocityMagnitudeMinMps,
            double allowableVelocityMagnitudeMaxMps) {}

    double fuelToFlywheelVelocity(double fuelVelocityMps);

    double flywheelToFuelVelocity(double flywheelVelocityMps);

    ShotCalculation calculateShot(double distanceToTargetMeters, double robotVelocityTowardsTargetMps);
}
