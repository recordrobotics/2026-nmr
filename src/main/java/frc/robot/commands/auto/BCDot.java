package frc.robot.commands.auto;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.List;
import java.util.Optional;

public class BCDot extends Command {

    public static final Pose2d BC_DOT_BLUE_DEPOT = new Pose2d(FlippingUtil.fieldSizeX / 2, 7.408, Rotation2d.kZero);
    public static final Pose2d BC_DOT_BLUE_OUTPOST = new Pose2d(FlippingUtil.fieldSizeX / 2, 0.713, Rotation2d.kZero);
    public static final Pose2d BC_DOT_RED_DEPOT = FlippingUtil.flipFieldPose(BC_DOT_BLUE_DEPOT);
    public static final Pose2d BC_DOT_RED_OUTPOST = FlippingUtil.flipFieldPose(BC_DOT_BLUE_OUTPOST);

    private PIDController xPid = new PIDController(3.0, 0.0, 0.15);
    private PIDController yPid = new PIDController(3.0, 0.0, 0.15);
    private PIDController rPid = new PIDController(4.0, 0.0, 0.3);

    private Optional<Pose2d> pidTarget = Optional.empty();

    public BCDot() {
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void initialize() {
        // if we decide to profile reset here
        Translation2d robotPose =
                RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation();
        pidTarget = List.of(BC_DOT_BLUE_DEPOT, BC_DOT_BLUE_OUTPOST, BC_DOT_RED_DEPOT, BC_DOT_RED_OUTPOST).stream()
                .sorted((a, b) -> Double.compare(
                        a.getTranslation().getSquaredDistance(robotPose),
                        b.getTranslation().getSquaredDistance(robotPose)))
                .findFirst();
    }

    @Override
    public void execute() {
        if (pidTarget.isEmpty()) return;
        Pose2d pidTargetVal = pidTarget.get();
        Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        double x = xPid.calculate(robotPose.getX(), pidTargetVal.getX());
        double y = yPid.calculate(robotPose.getY(), pidTargetVal.getY());
        double r = rPid.calculate(
                robotPose.getRotation().getRadians(), pidTargetVal.getRotation().getRadians());

        AutoControlModifier.getDefault()
                .drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, r), robotPose.getRotation()),
                        new double[0],
                        new double[0]);
    }

    @Override
    public boolean isFinished() {
        return pidTarget.isEmpty(); // nope we keep trying unless no target :(
    }
}
