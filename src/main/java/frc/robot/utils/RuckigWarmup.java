package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RuckigAlign;
import frc.robot.commands.WaypointAlign;
import frc.robot.commands.WaypointAlign.KinematicConstraints;
import frc.robot.utils.modifiers.AutoControlModifier;
import frc.robot.utils.modifiers.ControlModifierService;
import frc.robot.utils.modifiers.DrivetrainControl;
import java.util.List;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public final class RuckigWarmup {

    private static final List<Pose2d> WAYPOINTS = List.of(
            new Pose2d(0, 0, Rotation2d.k180deg),
            new Pose2d(2, 0, Rotation2d.kCCW_90deg),
            new Pose2d(1, 2, Rotation2d.kZero));

    private static final Double[] WAYPOINT_TIMEOUTS = {5.0, 5.0, 5.0};

    private RuckigWarmup() {}

    public static Command warmupCommand() {
        return WaypointAlign.alignWithCommand(
                        WAYPOINTS,
                        WAYPOINT_TIMEOUTS,
                        0,
                        1,
                        Commands.none(),
                        KinematicConstraints.DEFAULT,
                        EmptyAutoControlModifier.getDefault(),
                        EmptyAutoControlModifier.getDefault()::getCurrentDrivetrainKinematicState)
                .andThen(() -> ConsoleLogger.logInfo("Ruckig warmup complete"))
                .ignoringDisable(true);
    }

    @SuppressWarnings("java:S6548") // Singleton for default instance
    private static final class EmptyAutoControlModifier extends AutoControlModifier {
        private static EmptyAutoControlModifier defaultInstance;

        private Pose2d currentPose = new Pose2d();
        private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

        private EmptyAutoControlModifier() {}

        @SuppressWarnings("EffectivelyPrivate")
        public static synchronized EmptyAutoControlModifier getDefault() {
            if (defaultInstance == null) {
                defaultInstance = ControlModifierService.getInstance()
                        .createModifier(EmptyAutoControlModifier::new, Priority.AUTO);
            }
            return defaultInstance;
        }

        @Override
        protected boolean applyChassisSpeeds(
                ChassisSpeeds speeds,
                double[] robotRelativeForcesXNewtons,
                double[] robotRelativeForcesYNewtons,
                DrivetrainControl control) {

            // convert input robot-relative speeds to field-relative for odometry
            speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, currentPose.getRotation());

            currentSpeeds = speeds;
            double[] position = RuckigAlign.getCurrentNewPosition();
            currentPose = new Pose2d(position[0], position[1], new Rotation2d(position[2]));

            return true;
        }

        private KinematicState getCurrentDrivetrainKinematicState() {
            return RuckigAlign.toKinematicState(currentPose, currentSpeeds, new ChassisSpeeds());
        }
    }
}
