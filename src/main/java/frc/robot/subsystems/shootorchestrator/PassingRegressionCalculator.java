package frc.robot.subsystems.shootorchestrator;

@SuppressWarnings("java:S109") /* whole class is a magic number */
public class PassingRegressionCalculator implements ShotCalculator {

    @Override
    public double fuelToFlywheelVelocity(double fuelVelocityMps) {
        double ratio = HubRegressionCalculator.flywheelRatio.get();
        return fuelVelocityMps / ratio;
    }

    @Override
    public double flywheelToFuelVelocity(double flywheelVelocityMps) {
        double ratio = HubRegressionCalculator.flywheelRatio.get();
        return flywheelVelocityMps * ratio;
    }

    @Override
    public ShotCalculation calculateShot(double distanceToTargetMeters, double robotVelocityTowardsTargetMps) {
        return new ShotCalculation(
                Math.toRadians(calculateAngle(distanceToTargetMeters, robotVelocityTowardsTargetMps)),
                calculateBallVelocity(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                calculateTimeOfFlight(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                Math.toRadians(calculateAllowableShootAngleMin(distanceToTargetMeters, robotVelocityTowardsTargetMps)),
                Math.toRadians(calculateAllowableShootAngleMax(distanceToTargetMeters, robotVelocityTowardsTargetMps)),
                calculateAllowableVelocityMagnitudeMin(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                calculateAllowableVelocityMagnitudeMax(distanceToTargetMeters, robotVelocityTowardsTargetMps));
    }

    public static double calculateAngle(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 49.859734928
                + -0.272845617732 * robotVelocity
                + -0.991148634043 * robotVelocity2
                + -0.0826117216629 * robotVelocity3
                + 0.00638104079757 * robotVelocity4
                + 2.86813932138 * distance
                + 1.30028666575 * distance * robotVelocity
                + 0.195448844706 * distance * robotVelocity2
                + 0.00557291128529 * distance * robotVelocity3
                + -0.689082931181 * distance2
                + -0.157559202891 * distance2 * robotVelocity
                + -0.009739821626 * distance2 * robotVelocity2
                + 0.0482569691739 * distance3
                + 0.00515810131838 * distance3 * robotVelocity
                + -0.00104220022003 * distance4;
    }

    public static double calculateBallVelocity(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 2.22626774077
                + -0.687635405318 * robotVelocity
                + 0.0310073919734 * robotVelocity2
                + -0.00100008820402 * robotVelocity3
                + -0.000116678815344 * robotVelocity4
                + 1.42324141334 * distance
                + -0.00539080634384 * distance * robotVelocity
                + -0.000237261757245 * distance * robotVelocity2
                + -4.59666269709e-06 * distance * robotVelocity3
                + -0.118718817623 * distance2
                + 0.000450591337966 * distance2 * robotVelocity
                + -3.72720592586e-05 * distance2 * robotVelocity2
                + 0.00705262442111 * distance3
                + -1.65383210993e-05 * distance3 * robotVelocity
                + -0.000163195976748 * distance4;
    }

    public static double calculateTimeOfFlight(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 0.318176436189
                + -0.126179623528 * robotVelocity
                + -0.0162401043712 * robotVelocity2
                + -0.00123084197628 * robotVelocity3
                + 0.000114401352533 * robotVelocity4
                + 0.294410686104 * distance
                + 0.0239731528507 * distance * robotVelocity
                + 0.00374675565053 * distance * robotVelocity2
                + 5.21785854748e-05 * distance * robotVelocity3
                + -0.0354196665062 * distance2
                + -0.00265585289049 * distance2 * robotVelocity
                + -0.000200235259338 * distance2 * robotVelocity2
                + 0.00242466018199 * distance3
                + 8.7490922569e-05 * distance3 * robotVelocity
                + -5.8036960984e-05 * distance4;
    }

    // TODO: add regression
    public static double calculateAllowableShootAngleMin(double distance, double robotVelocity) {
        return -5;
    }

    public static double calculateAllowableShootAngleMax(double distance, double robotVelocity) {
        return 5;
    }

    public static double calculateAllowableVelocityMagnitudeMin(double distance, double robotVelocity) {
        return -3.89;
    }

    public static double calculateAllowableVelocityMagnitudeMax(double distance, double robotVelocity) {
        return 3.89;
    }
}
