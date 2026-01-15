package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ProjectileSimulationUtils {
    private ProjectileSimulationUtils() {}

    /**
     *
     *
     * <h2>Calculates the Initial Velocity of the Game Piece Projectile in the X-Y Plane.</h2>
     *
     * <p>This method calculates the initial velocity of the game piece projectile, accounting for the
     * chassis's translational and rotational motion as well as the shooter's ground speed.
     *
     * @param shooterPositionOnRobot the translation of the shooter on the robot, in the robot's frame
     *     of reference
     * @param chassisSpeeds the speeds of the chassis when the game piece is launched, including
     *     translational and rotational velocities
     * @param chassisFacing the direction the chassis is facing at the time of the launch
     * @param groundSpeedMPS the ground component of the projectile's initial velocity, provided as a
     *     scalar in meters per second (m/s)
     * @return the calculated initial velocity of the projectile as a {@link Translation2d} in meters
     *     per second
     */
    public static Translation2d calculateInitialProjectileVelocityMPS(
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d chassisFacing,
            double groundSpeedMPS) {
        final Translation2d chassisTranslationalVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final Translation2d shooterGroundVelocityDueToChassisRotation = shooterPositionOnRobot
                .rotateBy(chassisFacing)
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeeds.omegaRadiansPerSecond);
        final Translation2d shooterGroundVelocity =
                chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
    }

    public static Translation2d calculateInitialProjectileVelocityMPS(
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d chassisFacing,
            Translation2d groundSpeedMPS) {
        final Translation2d chassisTranslationalVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final Translation2d shooterGroundVelocityDueToChassisRotation = shooterPositionOnRobot
                .rotateBy(chassisFacing)
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeeds.omegaRadiansPerSecond);
        final Translation2d shooterGroundVelocity =
                chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(groundSpeedMPS.rotateBy(chassisFacing));
    }
}
