package frc.robot.utils.maplesim;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import java.util.Arrays;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class BumpSim {
    public record DoublePair(double first, double second) {}

    public record BumpLine(Translation3d start, Translation3d end) {}

    static final BumpLine[] BUMP_LINES = {
        new BumpLine(
                new Translation3d(4.004176, 1.583531, 0),
                new Translation3d(4.625626, FlippingUtil.fieldSizeY - 1.583531, 0.166517)),
        new BumpLine(
                new Translation3d(4.625626, 1.583531, 0.166517),
                new Translation3d(5.247075, FlippingUtil.fieldSizeY - 1.583531, 0)),
        new BumpLine(
                new Translation3d(FlippingUtil.fieldSizeX - 5.247075, 1.583531, 0),
                new Translation3d(FlippingUtil.fieldSizeX - 4.625626, FlippingUtil.fieldSizeY - 1.583531, 0.166517)),
        new BumpLine(
                new Translation3d(FlippingUtil.fieldSizeX - 4.625626, 1.583531, 0.166517),
                new Translation3d(FlippingUtil.fieldSizeX - 4.004176, FlippingUtil.fieldSizeY - 1.583531, 0)),
    };

    /**
     * Returns the height of the center of the sphere, and the effective slope, which is perpendicular to the line through the center of the sphere and its contact point with the ramp or endpoint. if the fuel is resting on the ground, this is the
     * radius above the ground and 0. however, if the sphere is on the ramp, this is the height above the ramp
     * surface (or, if resting on a ramp's tip, tangent to that endpoint). */
    public static DoublePair getSphereHeightAndEffectiveSlope(Translation2d sphereTranslationd, double sphereRadius) {

        double sphereX = sphereTranslationd.getX();
        double sphereY = sphereTranslationd.getY();

        // first pass to find where balls should be resting on the ramp itself
        for (int i = 0; i < BUMP_LINES.length; i++) {
            BumpLine line = BUMP_LINES[i];
            Translation3d lineStart = line.start();
            Translation3d lineEnd = line.end();

            if (sphereY < lineStart.getY() || sphereY > lineEnd.getY()) continue;

            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            Translation2d startOfRampToSphereCenter = new Translation2d(sphereX, sphereRadius).minus(start2d);
            double projectionT = startOfRampToSphereCenter.dot(lineVec) / lineVec.getSquaredNorm();

            if (projectionT < 0 || projectionT > 1)
                continue; // sphere is not flat on the ramp, possibly could be accounted for in second pass

            // sphere projects onto the segment itself - rest tangent to the line
            Translation2d projected = start2d.plus(lineVec.times(projectionT));

            double dx = sphereX - projected.getX();
            if (Math.abs(dx) >= sphereRadius) continue; // too far horizontally to touch this segment

            double dz = Math.sqrt(sphereRadius * sphereRadius - dx * dx);
            if (projected.getY() + dz > sphereRadius) /* never put sphere below ground */
                return new DoublePair(
                        projected.getY() + dz, (end2d.getY() - start2d.getY()) / (end2d.getX() - start2d.getX()));
        }

        // second pass to find where balls should be resting on the ramp's tips (endpoints)
        for (int i = 0; i < BUMP_LINES.length; i++) {
            BumpLine line = BUMP_LINES[i];
            Translation3d lineStart = line.start();
            Translation3d lineEnd = line.end();

            if (sphereY < lineStart.getY() || sphereY > lineEnd.getY()) continue;

            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            Translation2d toSphere = new Translation2d(sphereX, sphereRadius).minus(start2d);
            double projectionT = toSphere.dot(lineVec) / lineVec.getSquaredNorm();

            if (projectionT < 0 || projectionT > 1) {
                // sphere's X is beyond the segment's extent - check if it's resting on the near endpoint (tip) instead
                Translation2d endpoint = projectionT < 0 ? start2d : end2d;

                double dx = sphereX - endpoint.getX();
                if (Math.abs(dx) >= sphereRadius) continue; // too far horizontally to touch this tip

                double dz = Math.sqrt(sphereRadius * sphereRadius - dx * dx);
                if (endpoint.getY() + dz > sphereRadius) {
                    /* never put sphere below ground */
                    Translation2d sphereCenterPosXZ = new Translation2d(sphereX, endpoint.getY() + dz);
                    Translation2d fromEndpointToSphere = sphereCenterPosXZ.minus(endpoint);
                    // the effective slope is perpendicular to the line through the center of the sphere and its contact
                    // point (which is the endpoint), therefore, an inverse reciprocal is used to find the perpendicular
                    // slope.
                    double effectiveSlope = -fromEndpointToSphere.getX() / fromEndpointToSphere.getY();
                    return new DoublePair(endpoint.getY() + dz, effectiveSlope);
                }
            }
        }

        return new DoublePair(sphereRadius, 0.0);
    }

    public static Translation3d updateFuel(ImprovedRebuiltFuelOnField fuel) {
        Translation2d fuelPose = fuel.getPoseOnField().getTranslation();

        DoublePair fuelHeightAndSlope = getSphereHeightAndEffectiveSlope(fuelPose, Constants.Game.FUEL_RADIUS_METERS);
        double fuelEffectiveSlope = fuelHeightAndSlope.second();

        double horizontalAcceleration = -9.81 * fuelEffectiveSlope / (1 + fuelEffectiveSlope * fuelEffectiveSlope);

        // F = ma
        fuel.applyForce(new Force(horizontalAcceleration * Constants.Game.FUEL_MASS_KG, 0.0));

        return new Translation3d(fuelPose.getX(), fuelPose.getY(), fuelHeightAndSlope.first());
    }

    /**
     * Updates the swerve drive simulation based on the current state of the robot and the module offsets.
     *
     * @param swerveDriveSimultion The swerve drive simulation to update.
     * @param moduleOffsets        Robot-relative module positions in order FL, FR, BL, BR (metres).
     * @param wheelRadiusMeters    The radius of the wheels in metres.
     * @return The updated pose of the swerve drive.
     */
    public static Pose3d updateSwerveDriveSimulation(
            SwerveDriveSimulation swerveDriveSimultion, Translation2d[] moduleOffsets, double wheelRadiusMeters) {
        Pose2d robotPose = swerveDriveSimultion.getSimulatedDriveTrainPose();
        double[] moduleZs = new double[4];
        for (int i = 0; i < moduleOffsets.length; i++) {
            Translation2d moduleOffset = moduleOffsets[i];
            Translation2d modulePosition = robotPose
                    .plus(new Transform2d(moduleOffset, Rotation2d.kZero))
                    .getTranslation();
            DoublePair moduleHeightAndSlope = getSphereHeightAndEffectiveSlope(modulePosition, wheelRadiusMeters);
            double moduleEffectiveSlope = moduleHeightAndSlope.second();

            double horizontalAcceleration =
                    -9.81 * moduleEffectiveSlope / (1 + moduleEffectiveSlope * moduleEffectiveSlope);

            moduleZs[i] = moduleHeightAndSlope.first();

            // F = ma
            swerveDriveSimultion.applyForce(
                    new Vector2(horizontalAcceleration * Constants.Frame.ROBOT_MASS_KG, 0.0),
                    new Vector2(modulePosition.getX(), modulePosition.getY()));
        }

        // FL=0, FR=1, BL=2, BR=3
        // these are the measurements of the rectangle formed by the four modules
        double moduleRectangleXDist = Math.max(Math.abs(moduleOffsets[0].getX() - moduleOffsets[2].getX()), 1e-3);
        double moduleRectangleYDist = Math.max(Math.abs(moduleOffsets[0].getY() - moduleOffsets[1].getY()), 1e-3);

        double roll = Math.atan2(moduleZs[0] + moduleZs[2] - moduleZs[1] - moduleZs[3], 2 * moduleRectangleYDist);
        double pitch = Math.atan2(moduleZs[2] + moduleZs[3] - moduleZs[0] - moduleZs[1], 2 * moduleRectangleXDist);

        return new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                Arrays.stream(moduleZs).sum() / 4.0 - wheelRadiusMeters,
                new Rotation3d(roll, pitch, robotPose.getRotation().getRadians()));
    }
}
