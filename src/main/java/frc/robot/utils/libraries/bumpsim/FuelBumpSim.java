package frc.robot.utils.libraries.bumpsim;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.utils.maplesim.ImprovedRebuiltFuelOnField;
import org.dyn4j.dynamics.Force;

public class FuelBumpSim {
    public record DoublePair<F, S>(F first, S second) {}

    private static final double FIELD_LENGTH = 16.51; // m

    private static final double FIELD_WIDTH = 8.04; // m

    /** Start points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_STARTS = {
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    };

    /** End points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_ENDS = {
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    };

    public static DoublePair<Double, Double> getFuelHeightAndHorizontalDisplacement(Translation2d fuelTranslationd) {
        /* returns the height of the center of the fuel, and the horizontal displacement from the fuel to its contact point with the ramp in the X direction on the fuel. if the fuel is resting on the ground, this is the
        radius above the ground and 0. however, if the fuel is on the ramp, this is the height above the ramp
        surface (or, if resting on a ramp's tip, tangent to that endpoint). */
        double fuelX = fuelTranslationd.getX();
        double fuelY = fuelTranslationd.getY();

        for (int i = 0; i < BUMP_LINE_STARTS.length; i++) {
            Translation3d lineStart = BUMP_LINE_STARTS[i];
            Translation3d lineEnd = BUMP_LINE_ENDS[i];

            if (fuelY < lineStart.getY() || fuelY > lineEnd.getY()) continue;

            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            Translation2d toFuel = new Translation2d(fuelX, Constants.Game.FUEL_RADIUS_METERS).minus(start2d);
            double projectionT = toFuel.dot(lineVec) / lineVec.getSquaredNorm();

            if (projectionT < 0 || projectionT > 1) {
                // fuel's X is beyond the segment's extent - check if it's resting on the near endpoint (tip) instead
                Translation2d endpoint = projectionT < 0 ? start2d : end2d;

                double dx = fuelX - endpoint.getX();
                if (Math.abs(dx) >= Constants.Game.FUEL_RADIUS_METERS)
                    continue; // too far horizontally to touch this tip

                double dz = Math.sqrt(Constants.Game.FUEL_RADIUS_METERS * Constants.Game.FUEL_RADIUS_METERS - dx * dx);
                if (endpoint.getY() + dz > Constants.Game.FUEL_RADIUS_METERS)
                    return new DoublePair<>(endpoint.getY() + dz, dx); // never put fuel below ground
                continue;
            }

            // fuel projects onto the segment itself - rest tangent to the line
            Translation2d projected = start2d.plus(lineVec.times(projectionT));

            double dx = fuelX - projected.getX();
            if (Math.abs(dx) >= Constants.Game.FUEL_RADIUS_METERS)
                continue; // too far horizontally to touch this segment

            double dz = Math.sqrt(Constants.Game.FUEL_RADIUS_METERS * Constants.Game.FUEL_RADIUS_METERS - dx * dx);
            if (projected.getY() + dz > Constants.Game.FUEL_RADIUS_METERS)
                return new DoublePair<>(projected.getY() + dz, dx); // never put fuel below ground
        }

        return new DoublePair<>(Constants.Game.FUEL_RADIUS_METERS, 0.0);
    }

    public static Translation3d updateFuel(ImprovedRebuiltFuelOnField fuel) {
        Translation2d fuelPose = fuel.getPoseOnField().getTranslation();

        double fuelX = fuelPose.getX();
        double fuelY = fuelPose.getY();

        double horizontalAcceleration = 0.0;

        for (int i = 0; i < BUMP_LINE_STARTS.length; i++) {
            Translation3d lineStart = BUMP_LINE_STARTS[i];
            Translation3d lineEnd = BUMP_LINE_ENDS[i];

            if (fuelY < lineStart.getY() || fuelY > lineEnd.getY()) {
                continue;
            }

            double x1 = lineStart.getX();
            double z1 = lineStart.getZ();
            double x2 = lineEnd.getX();
            double z2 = lineEnd.getZ();

            double dx = x2 - x1;
            double dz = z2 - z1;

            double lengthSquared = dx * dx + dz * dz;

            // Determine whether the ball's X position is horizontally
            // within this ramp segment.
            double t = (fuelX - x1) / dx;

            if (t < 0.0 || t > 1.0) {
                continue;
            }

            /*
             * Gravity along the ramp:
             *
             *   a = g * sin(theta)
             *
             * Horizontal component:
             *
             *   ax = g * sin(theta) * cos(theta)
             *
             * For a ramp vector (dx, dz):
             *
             *   sin(theta) = dz / sqrt(dx² + dz²)
             *   cos(theta) = dx / sqrt(dx² + dz²)
             *
             * Therefore:
             *
             *   ax = -g * dx * dz / (dx² + dz²)
             *
             * The negative sign makes the acceleration point downhill.
             */
            horizontalAcceleration = -9.81 * dx * dz / lengthSquared;

            break;
        }

        // F = ma
        fuel.applyForce(new Force(horizontalAcceleration * Constants.Game.FUEL_MASS_KG, 0.0));

        double height = getFuelHeightAndHorizontalDisplacement(fuelPose).first();

        return new Translation3d(fuelX, fuelY, height);
    }
}
