package frc.robot.utils.modifiers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * A control modifier that simply drives with the given ChassisSpeeds for this one periodic cycle.
 * Used for autonomous driving.
 */
@SuppressWarnings("java:S6548") // Singleton for default instance
public class RotationOverrideControlModifier extends OneshotControlModifier {

    private static RotationOverrideControlModifier defaultInstance;

    private ProfiledPIDController spinController = new ProfiledPIDController(
            Constants.Control.SPIN_KP, 0, Constants.Control.SPIN_KD, Constants.Control.SPIN_CONSTRAINTS);

    private double lastEnabledTime = Timer.getFPGATimestamp();

    private double targetAngleFieldRelative;

    protected RotationOverrideControlModifier() {
        spinController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static synchronized RotationOverrideControlModifier getDefault() {
        if (defaultInstance == null) {
            defaultInstance = ControlModifierService.getInstance()
                    .createModifier(RotationOverrideControlModifier::new, Priority.OVERRIDE);
        }
        return defaultInstance;
    }

    public void drive(double targetAngleFieldRelative) {
        Logger.recordOutput("RotationOverrideModifierAngle", targetAngleFieldRelative);
        this.targetAngleFieldRelative = targetAngleFieldRelative;
        this.lastEnabledTime = Timer.getFPGATimestamp();
        this.setEnabled(true);
    }

    @Override
    protected final boolean perform(DrivetrainControl control) {
        return applyChassisSpeeds(targetAngleFieldRelative, control);
    }

    protected boolean applyChassisSpeeds(double targetAngleFieldRelative, DrivetrainControl control) {
        if (Timer.getFPGATimestamp() - lastEnabledTime > 0.1) {
            spinController.reset(
                    RobotContainer.poseSensorFusion
                            .getEstimatedPosition()
                            .getRotation()
                            .getRadians(),
                    RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond);
        }
        double spinOutput = spinController.calculate(
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians(),
                targetAngleFieldRelative);
        Logger.recordOutput("RotationOverrideModifierSpinOutput", spinOutput);
        control.applyWeightedVelocity(
                new Transform2d(
                        control.toChassisSpeeds().vxMetersPerSecond,
                        control.toChassisSpeeds().vyMetersPerSecond,
                        new Rotation2d(spinOutput)),
                1);
        control.applyFeedforwards(new double[4], new double[4]);
        return true;
    }
}
