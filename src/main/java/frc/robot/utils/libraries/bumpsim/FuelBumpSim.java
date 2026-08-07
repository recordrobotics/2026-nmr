package frc.robot.utils.libraries.bumpsim;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.utils.maplesim.ImprovedRebuiltFuelOnField;
import org.dyn4j.dynamics.Force;

public class FuelBumpSim {
    public record DoublePair(double first, double second) {}

    private static final double FIELD_LENGTH_M = FlippingUtil.fieldSizeX;

    private static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;

    /** Start points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_STARTS = {
        new Translation3d(4.004176, 1.583531, 0),
        new Translation3d(4.625626, 1.583531, 0.166517),
        new Translation3d(FIELD_LENGTH_M - 5.247075, 1.583531, 0),
        new Translation3d(FIELD_LENGTH_M - 4.625626, 1.583531, 0.166517),
    };

    /** End points of the bump line segments. */
    static final Translation3d[] BUMP_LINE_ENDS = {
        new Translation3d(4.625626, FIELD_WIDTH - 1.583531, 0.166517),
        new Translation3d(5.247075, FIELD_WIDTH - 1.583531, 0),
        new Translation3d(FIELD_LENGTH_M - 4.625626, FIELD_WIDTH - 1.583531, 0.166517),
        new Translation3d(FIELD_LENGTH_M - 4.004176, FIELD_WIDTH - 1.583531, 0),
    };

    public static DoublePair getFuelHeightAndEffectiveSlope(Translation2d fuelTranslationd) {
        /* Returns the height of the center of the fuel, and the effective slope, which is perpendicular to the line through the center of the fuel and its contact point with the ramp or endpoint. if the fuel is resting on the ground, this is the
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

            if (projectionT < 0 || projectionT > 1)
                continue; // fuel is not flat on the ramp, possibly could be accounted for in second pass

            // fuel projects onto the segment itself - rest tangent to the line
            Translation2d projected = start2d.plus(lineVec.times(projectionT));

            double dx = fuelX - projected.getX();
            if (Math.abs(dx) >= Constants.Game.FUEL_RADIUS_METERS)
                continue; // too far horizontally to touch this segment

            double dz = Math.sqrt(Constants.Game.FUEL_RADIUS_METERS * Constants.Game.FUEL_RADIUS_METERS - dx * dx);
            if (projected.getY() + dz > Constants.Game.FUEL_RADIUS_METERS) /* never put fuel below ground */
                return new DoublePair(
                        projected.getY() + dz, (end2d.getY() - start2d.getY()) / (end2d.getX() - start2d.getX()));
        }

        for (int i = 0;
                i < BUMP_LINE_STARTS.length;
                i++) { // second pass to find where balls should be resting on the endpoint of a ramp
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
                if (endpoint.getY() + dz > Constants.Game.FUEL_RADIUS_METERS) {
                    /* never put fuel below ground */
                    Translation2d fuelCenterPosXZ = new Translation2d(fuelX, endpoint.getY() + dz);
                    Translation2d fromEndpointToFuel = fuelCenterPosXZ.minus(endpoint);
                    // the effective slope is perpendicular to the line through the center of the fuel and its contact
                    // point (which is the endpoint), therefore, an inverse reciprical is used to find the perpendicular
                    // slope.
                    double effectiveSlope = -fromEndpointToFuel.getX() / fromEndpointToFuel.getY();
                    return new DoublePair(endpoint.getY() + dz, effectiveSlope);
                }
                continue;
            }
        }

        return new DoublePair(Constants.Game.FUEL_RADIUS_METERS, 0.0);
    }

    public static Translation3d updateFuel(ImprovedRebuiltFuelOnField fuel) {
        Translation2d fuelPose = fuel.getPoseOnField().getTranslation();

        DoublePair fuelHeightAndSlope = getFuelHeightAndEffectiveSlope(fuelPose);
        double fuelEffectiveSlope = fuelHeightAndSlope.second();

        double horizontalAcceleration = -9.81 * fuelEffectiveSlope / (1 + fuelEffectiveSlope * fuelEffectiveSlope);

        // F = ma
        fuel.applyForce(new Force(horizontalAcceleration * Constants.Game.FUEL_MASS_KG, 0.0));

        return new Translation3d(fuelPose.getX(), fuelPose.getY(), fuelHeightAndSlope.first());
    }
}
