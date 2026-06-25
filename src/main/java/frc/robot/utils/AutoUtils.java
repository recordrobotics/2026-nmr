package frc.robot.utils;

import frc.robot.RobotContainer;
import frc.robot.commands.RuckigAlign;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public final class AutoUtils {
    private AutoUtils() {}

    public static KinematicState getCurrentDrivetrainKinematicState() {
        return RuckigAlign.toKinematicState(
                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                RobotContainer.drivetrain.getChassisSpeeds(),
                RobotContainer.drivetrain.getChassisAcceleration());
    }
}
