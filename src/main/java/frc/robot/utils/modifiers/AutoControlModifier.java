package frc.robot.utils.modifiers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * A control modifier that simply drives with the given ChassisSpeeds for this one periodic cycle.
 * Used for autonomous driving.
 */
@SuppressWarnings("java:S6548") // Singleton for default instance
public class AutoControlModifier extends OneshotControlModifier {

    private static AutoControlModifier defaultInstance;

    private ChassisSpeeds speeds;
    private double[] robotRelativeForcesXNewtons;
    private double[] robotRelativeForcesYNewtons;

    protected AutoControlModifier() {}

    public static synchronized AutoControlModifier getDefault() {
        if (defaultInstance == null) {
            defaultInstance =
                    ControlModifierService.getInstance().createModifier(AutoControlModifier::new, Priority.AUTO);
        }
        return defaultInstance;
    }

    public void drive(
            ChassisSpeeds speeds, double[] robotRelativeForcesXNewtons, double[] robotRelativeForcesYNewtons) {
        Logger.recordOutput("AUTOSPEEDS", speeds);
        this.speeds = speeds;
        this.robotRelativeForcesXNewtons = robotRelativeForcesXNewtons;
        this.robotRelativeForcesYNewtons = robotRelativeForcesYNewtons;
        this.setEnabled(true);
    }

    @Override
    protected final boolean perform(DrivetrainControl control) {
        return applyChassisSpeeds(speeds, robotRelativeForcesXNewtons, robotRelativeForcesYNewtons, control);
    }

    protected boolean applyChassisSpeeds(
            ChassisSpeeds speeds,
            double[] robotRelativeForcesXNewtons,
            double[] robotRelativeForcesYNewtons,
            DrivetrainControl control) {
        control.applyWeightedVelocity(
                new Transform2d(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        new Rotation2d(speeds.omegaRadiansPerSecond)),
                1);
        control.applyFeedforwards(robotRelativeForcesXNewtons, robotRelativeForcesYNewtons);
        return true;
    }
}
