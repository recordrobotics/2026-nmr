package frc.robot.subsystems.shootorchestrator;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@SuppressWarnings("java:S109") /* whole class is a magic number */
public class HubRegressionCalculator implements ShotCalculator {

    private static final double FLYWHEEL_TO_FUEL_RATIO = 0.372;

    public static final LoggedNetworkNumber flywheelRatio =
            new LoggedNetworkNumber("FLYWHEEL_RATIO", FLYWHEEL_TO_FUEL_RATIO);

    @Override
    public double fuelToFlywheelVelocity(double fuelVelocityMps) {
        double ratio = flywheelRatio.get();
        return fuelVelocityMps / ratio;
    }

    @Override
    public double flywheelToFuelVelocity(double flywheelVelocityMps) {
        double ratio = flywheelRatio.get();
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
        return 95.0476215151
                + 7.22781779628 * robotVelocity
                + 0.446187166766 * robotVelocity2
                + -0.143025576382 * robotVelocity3
                + -0.0178376322206 * robotVelocity4
                + -25.4495442603 * distance
                + 0.27772525426 * distance * robotVelocity
                + 0.0236510086091 * distance * robotVelocity2
                + 0.0194862031565 * distance * robotVelocity3
                + 6.92921040811 * distance2
                + -0.186088818887 * distance2 * robotVelocity
                + -0.00120770883884 * distance2 * robotVelocity2
                + -0.89854631409 * distance3
                + 0.0108330719224 * distance3 * robotVelocity
                + 0.0416242296937 * distance4;
    }

    public static double calculateBallVelocity(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 6.36807950256
                + -0.318415367375 * robotVelocity
                + 0.0568602191061 * robotVelocity2
                + -0.00155522629749 * robotVelocity3
                + -0.000626428266815 * robotVelocity4
                + -0.791430248818 * distance
                + -0.119230385626 * distance * robotVelocity
                + -0.0024168567017 * distance * robotVelocity2
                + 0.000168717081825 * distance * robotVelocity3
                + 0.51592576216 * distance2
                + 0.0212946084093 * distance2 * robotVelocity
                + -5.69927539992e-05 * distance2 * robotVelocity2
                + -0.0741889239284 * distance3
                + -0.00147749738957 * distance3 * robotVelocity
                + 0.00363337018461 * distance4;
    }

    public static double calculateTimeOfFlight(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 1.31892611933
                + -0.0683937405325 * robotVelocity
                + 0.00374486350338 * robotVelocity2
                + -0.00193237414201 * robotVelocity3
                + -0.000338254753748 * robotVelocity4
                + -0.223678576703 * distance
                + 0.0326917274421 * distance * robotVelocity
                + 0.00123375588771 * distance * robotVelocity2
                + 0.000284834497372 * distance * robotVelocity3
                + 0.0980559153379 * distance2
                + -0.00570370842433 * distance2 * robotVelocity
                + -0.000115350359524 * distance2 * robotVelocity2
                + -0.0128365409099 * distance3
                + 0.000277738089997 * distance3 * robotVelocity
                + 0.000560687425104 * distance4;
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
