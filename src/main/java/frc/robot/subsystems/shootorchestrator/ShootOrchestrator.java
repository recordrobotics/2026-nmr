package frc.robot.subsystems.shootorchestrator;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Spindexer.SpindexerState;
import frc.robot.subsystems.Turret.TurretState;
import frc.robot.subsystems.shootorchestrator.ShotCalculator.ShotCalculation;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PositionedSubsystem.PositionStatus;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.field.FieldUtils;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShootOrchestrator extends ManagedSubsystemBase {

    private static final ShotCalculator hubCalculator = new HubRegressionCalculator();
    private static final ShotCalculator passingCalculator = new PassingRegressionCalculator();

    private static final double BOTTOM_BEAM_TIME_TO_BALL_HIT = 0.06;
    private static final double TOP_BEAM_TIME_TO_BALL_HIT = 0.04;

    private static final ShotCalculation FIXED_SHOT_CALCULATION = hubCalculator.calculateShot(2.944349, 0);
    private static final double FIXED_TURRET_ANGLE_RADIANS = Units.degreesToRadians(90);
    private static final double FIXED_HOOD_ANGLE_RADIANS =
            FIXED_SHOT_CALCULATION.shootAngleRadians() - Constants.Shooter.HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS;
    private static final double FIXED_FUEL_VELOCITY = FIXED_SHOT_CALCULATION.fuelVelocityMagnitudeMps();

    private static final LoggedNetworkNumber shotFeedforward = new LoggedNetworkNumber("SHOTFEED", 1.224808013371447);
    private static final LoggedNetworkNumber hoodAngleDashboardOverride =
            new LoggedNetworkNumber("HOOD_ANGLE", Constants.Shooter.HOOD_MAX_POSITION_RADIANS);
    private static final LoggedNetworkNumber shootVelocityDashboardOverride =
            new LoggedNetworkNumber("SHOOT_VELOCITY", 0);
    private static final LoggedNetworkBoolean shootOverride = new LoggedNetworkBoolean("SHOOT_OVERRIDE", false);
    private static final LoggedNetworkNumber shootAngleOffset = new LoggedNetworkNumber("SHOOT_ANGLE_OFFSET", 0);

    private static final LoggedDashboardChooser<FeedForwardSource> feedForwardSourceChooser =
            new LoggedDashboardChooser<>("FeedForwardSource");

    public enum FeedMode {
        AUTO,
        ALWAYS,
        DISABLED
    }

    public enum FeedForwardSource {
        NONE("None"),
        BEAM_BREAKS("Beam Breaks");

        private final String name;

        FeedForwardSource(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    private Optional<ShooterState> shooterOverride = Optional.empty();

    private FeedMode feedMode = FeedMode.AUTO;

    private Optional<ShotTarget> target = Optional.empty();
    private boolean shootingEnabled = false;
    private boolean fixedMode = false;
    private boolean useFixedShooting = false;

    private OptionalDouble lastShotYaw = OptionalDouble.empty();

    private double lastShotTimeOfFlight = 0;

    private double timeAtBallHit = 0;
    private double shooterFeedforward = 0;

    private boolean lastOnTarget = false;

    private SafeAlert shootOverrideAlert = new SafeAlert("Shooting override enabled!", AlertType.kWarning);
    private SafeAlert feedModeAlert = new SafeAlert("feed mode alert", AlertType.kWarning);

    public record ShotTarget(Translation3d position, ShotCalculator shotCalculator) {}

    public ShootOrchestrator() {
        feedForwardSourceChooser.addDefaultOption(FeedForwardSource.NONE.toString(), FeedForwardSource.NONE);
        for (FeedForwardSource source : FeedForwardSource.values()) {
            if (source != FeedForwardSource.NONE) {
                feedForwardSourceChooser.addOption(source.toString(), source);
            }
        }
    }

    public void setEnableShooting(boolean enable) {
        this.shootingEnabled = enable;
    }

    public boolean isShootingEnabled() {
        return shootingEnabled;
    }

    public void setFixedMode(boolean fixed) {
        this.fixedMode = fixed;
        if (!fixed) {
            useFixedShooting = false;
        }
    }

    public void setFeedMode(FeedMode mode) {
        this.feedMode = mode;
    }

    public final void setTarget(ShotTarget target) {
        this.target = Optional.of(target);
    }

    public void clearTarget() {
        this.target = Optional.empty();
    }

    public Optional<ShotTarget> getTarget() {
        return target;
    }

    public void setShooterOverride(ShooterState shooterState) {
        this.shooterOverride = Optional.of(shooterState);
    }

    public void clearShooterOverride() {
        this.shooterOverride = Optional.empty();
    }

    /**
     * Sets the target based on the robot's position on the field. If the robot is in the alliance zone, it will target the hub. Otherwise, it will target the appropriate passing target based on the robot's Y position. Also sets whether to use fixed shooting based on whether we're in the alliance zone and whether fixed mode is enabled.
     */
    private void setAutomatedTarget() {
        if (FieldUtils.isInAllianceZone()) {
            setTarget(new ShotTarget(
                    DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                            ? Constants.Game.BLUE_HUB_POSITION
                            : Constants.Game.RED_HUB_POSITION,
                    hubCalculator));
            useFixedShooting = fixedMode;
        } else {
            if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) {
                setTarget(new ShotTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() < FlippingUtil.fieldSizeY / 2
                                ? Constants.Game.BLUE_PASSING_TARGET_HP_SIDE
                                : Constants.Game.BLUE_PASSING_TARGET_DEPOT_SIDE,
                        passingCalculator));
            } else {
                setTarget(new ShotTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() > FlippingUtil.fieldSizeY / 2
                                ? Constants.Game.RED_PASSING_TARGET_HP_SIDE
                                : Constants.Game.RED_PASSING_TARGET_DEPOT_SIDE,
                        passingCalculator));
            }
            useFixedShooting = false;
        }
    }

    private void feedforwardShooterBeambreaks() {
        double currentTime = Timer.getTimestamp();

        if (RobotContainer.feeder.isBottomBeamBroken()) {
            timeAtBallHit = currentTime + BOTTOM_BEAM_TIME_TO_BALL_HIT;
        }

        if (RobotContainer.feeder.isTopBeamBroken()) {
            timeAtBallHit = currentTime + TOP_BEAM_TIME_TO_BALL_HIT;
        }

        if (timeAtBallHit > currentTime) {
            double timeLeft = timeAtBallHit - currentTime;
            if (timeLeft <= 0.06 && timeLeft >= 0.02) {
                shooterFeedforward = shotFeedforward.get();
            }
        } else {
            shooterFeedforward = 0;
        }
    }

    public double getShotTimeOfFlight() {
        return lastShotTimeOfFlight;
    }

    /**
     *
     * @param robotPose
     * @param robotRelativeSpeeds
     * @param shooterReleaseOffset robot relative offset from robot center where shots are released, used for calculating distance to target and velocity
     * @return
     */
    private ShotCalculationResult calculateShot(
            ShotTarget target,
            Pose3d robotPose,
            ChassisSpeeds robotRelativeSpeeds,
            Translation3d shooterReleaseOffset) {
        Translation3d shooterReleasePosition = robotPose
                .transformBy(new Transform3d(shooterReleaseOffset, Rotation3d.kZero))
                .getTranslation();

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds, robotPose.toPose2d().getRotation());

        double omega = robotSpeeds.omegaRadiansPerSecond;
        // Add robot velocity to velocity induced from angular rotation
        double releaseVx = robotSpeeds.vxMetersPerSecond - omega * shooterReleaseOffset.getY();
        double releaseVy = robotSpeeds.vyMetersPerSecond + omega * shooterReleaseOffset.getX();
        Transform2d shooterReleaseVelocity = new Transform2d(releaseVx, releaseVy, new Rotation2d(omega));

        Vector<N2> shooterReleaseVelocityVector =
                shooterReleaseVelocity.getTranslation().toVector();

        Translation2d relativeTarget =
                target.position.toTranslation2d().minus(shooterReleasePosition.toTranslation2d());

        double distanceToTarget = relativeTarget.getNorm();
        Vector<N2> normTargetVector = relativeTarget.toVector().div(distanceToTarget);
        double radialVelocity = shooterReleaseVelocityVector.dot(normTargetVector);

        Vector<N2> tangentialVelocity = shooterReleaseVelocityVector.minus(normTargetVector.times(radialVelocity));

        ShotCalculation shotCalculation = target.shotCalculator.calculateShot(distanceToTarget, radialVelocity);

        lastShotTimeOfFlight = shotCalculation.timeOfFlightSeconds();

        double targetYaw = Math.atan2(relativeTarget.getY(), relativeTarget.getX());
        double targetPitch = shotCalculation.shootAngleRadians();
        double targetSpeed = shotCalculation.fuelVelocityMagnitudeMps();

        // Construct a vector from yaw,pitch,and magnitude
        // and subtract tangential velocity (not compensated for by shot calculator)
        return new ShotCalculationResult(
                new Translation3d(targetSpeed, 0.0, 0.0)
                        .rotateBy(new Rotation3d(0.0, -targetPitch, targetYaw))
                        .toVector()
                        .minus(new Translation3d(new Translation2d(tangentialVelocity)).toVector()),
                shotCalculation);
    }

    private TurretState calculateTurretState(
            Vector<N3> fieldShotVector,
            Vector<N3> robotRelativeShotVector,
            ChassisSpeeds robotRelativeSpeeds,
            ChassisSpeeds robotRelativeAcceleration) {
        double shotYaw = Math.atan2(robotRelativeShotVector.get(1), robotRelativeShotVector.get(0));

        // Keep yaw derivative in field coordinates due to the bad way its computed instead of using actual derivative
        double fieldShotYaw = Math.atan2(fieldShotVector.get(1), fieldShotVector.get(0));
        double shotYawVelocity = lastShotYaw.isPresent()
                ? MathUtil.inputModulus(fieldShotYaw - lastShotYaw.getAsDouble(), -Math.PI, Math.PI) / 0.02
                : 0;
        lastShotYaw = OptionalDouble.of(fieldShotYaw);

        if (RobotContainer.intake.isNearStartPosition()
                || RobotContainer.intake.getTargetState() == IntakeState.STARTING) {
            double turretPos = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
            return new TurretState(Math.copySign(Constants.Turret.STARTING_POSITION_RADIANS, turretPos), 0, 0);
        } else if (useFixedShooting) {
            return new TurretState(FIXED_TURRET_ANGLE_RADIANS, 0, 0);
        } else {
            return new TurretState(
                    shotYaw,
                    shotYawVelocity - robotRelativeSpeeds.omegaRadiansPerSecond,
                    -robotRelativeAcceleration.omegaRadiansPerSecond);
        }
    }

    private double shootAngleToHoodAngle(double shootAngle) {
        return shootAngle - Constants.Shooter.HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS + shootAngleOffset.get();
    }

    private ShooterState calculateShooterState(
            ShotTarget target, Vector<N3> robotRelativeShotVector, boolean isBlocked) {
        if (shootingEnabled) {
            if (shootOverride.get()) {
                double hoodAngle = shooterOverride.isPresent()
                        ? shooterOverride.get().hoodAngleRadians()
                        : hoodAngleDashboardOverride.get();
                return new ShooterState(
                        isBlocked ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : hoodAngle,
                        shooterOverride.isPresent()
                                ? shooterOverride.get().flywheelVelocityMps()
                                : shootVelocityDashboardOverride.get(),
                        shooterFeedforward);
            } else {
                double hoodAngle = shootAngleToHoodAngle(Math.atan2(
                        robotRelativeShotVector.get(2),
                        Math.hypot(robotRelativeShotVector.get(0), robotRelativeShotVector.get(1))));

                if (useFixedShooting) {
                    hoodAngle = FIXED_HOOD_ANGLE_RADIANS;
                }

                return new ShooterState(
                        isBlocked ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : hoodAngle,
                        target.shotCalculator.fuelToFlywheelVelocity(
                                useFixedShooting ? FIXED_FUEL_VELOCITY : robotRelativeShotVector.norm()),
                        shooterFeedforward);
            }
        } else {
            return new ShooterState(Constants.Shooter.HOOD_MAX_POSITION_RADIANS, 0, 0);
        }
    }

    private boolean calculateIsHoodBlocked(
            Pose3d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation3d hoodPosition) {
        double timeUntilHoodDown =
                RobotContainer.shooter.getTimeUntilHoodAt(Constants.Shooter.HOOD_MAX_POSITION_RADIANS)
                        + 0.1 /* latency compensation */;

        Pose2d robotPoseWhenDown =
                SimpleMath.integrateChassisSpeeds(robotPose.toPose2d(), robotRelativeSpeeds, timeUntilHoodDown);
        Translation3d hoodPoseWhenDown = new Pose3d(robotPoseWhenDown)
                .transformBy(new Transform3d(hoodPosition, Rotation3d.kZero))
                .getTranslation();
        Translation3d hoodPoseCurrent = robotPose
                .transformBy(new Transform3d(hoodPosition, Rotation3d.kZero))
                .getTranslation();

        return FieldUtils.isBlocked(hoodPoseCurrent.toTranslation2d(), hoodPoseWhenDown.toTranslation2d());
    }

    private double calculateAllowableTurretError() {
        return Units.degreesToRadians(12); // TODO: add actual trig calc based on distance to target and radius
    }

    private boolean isOnTarget(ShotTarget target, ShotCalculation shotCalculation, boolean isBlocked) {
        boolean overridden = shootOverride.get();

        boolean shooterOnTarget;
        if (overridden) {
            shooterOnTarget = RobotContainer.shooter.isAtTargetState(Units.degreesToRadians(5), 10);
        } else {
            shooterOnTarget = RobotContainer.shooter.isAtTargetState(
                    shotCalculation.allowableShootAngleMinRadians(),
                    shotCalculation.allowableShootAngleMaxRadians(),
                    target.shotCalculator.fuelToFlywheelVelocity(shotCalculation.allowableVelocityMagnitudeMinMps()),
                    target.shotCalculator.fuelToFlywheelVelocity(shotCalculation.allowableVelocityMagnitudeMaxMps()));
        }

        return !isBlocked
                && RobotContainer.turret.atGoal(
                        overridden ? Units.degreesToRadians(12) : calculateAllowableTurretError())
                && shooterOnTarget
                && !(RobotContainer.intake.isNearStartPosition()
                        || RobotContainer.intake.getTargetState() == IntakeState.STARTING)
                && RobotContainer.turret.getPositionStatus() == PositionStatus.KNOWN
                && RobotContainer.shooter.getPositionStatus() == PositionStatus.KNOWN;
    }

    private void updateFeeders(boolean onTarget) {
        SpindexerState spindexerState =
                switch (feedMode) {
                    case AUTO -> (onTarget && shootingEnabled) ? SpindexerState.ON : SpindexerState.OFF;
                    case ALWAYS -> shootingEnabled ? SpindexerState.ON : SpindexerState.OFF;
                    case DISABLED -> SpindexerState.OFF;
                };
        FeederState feederState =
                switch (feedMode) {
                    case AUTO -> (onTarget && shootingEnabled) ? FeederState.ON : FeederState.OFF;
                    case ALWAYS -> shootingEnabled ? FeederState.ON : FeederState.OFF;
                    case DISABLED -> FeederState.OFF;
                };

        RobotContainer.spindexer.setState(
                RobotContainer.getControl().isUnstuckSpindexerPressed() ? SpindexerState.UNSTUCK : spindexerState);
        RobotContainer.feeder.setState(
                RobotContainer.getControl().isUnstuckSpindexerPressed() ? FeederState.UNSTUCK : feederState);
    }

    @Override
    public void periodicManaged() {
        updateAlerts();
        setAutomatedTarget();

        switch (Optional.ofNullable(feedForwardSourceChooser.get()).orElse(FeedForwardSource.NONE)) {
            case NONE -> shooterFeedforward = 0;
            case BEAM_BREAKS -> feedforwardShooterBeambreaks();
        }

        if (target.isPresent()) {
            ShotTarget shotTarget = target.get();

            Pose3d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition3d();
            Translation3d fuelReleaseOffset = RobotContainer.model.shooterModel.getShooterFuelReleasePosition();
            ChassisSpeeds robotRelativeSpeeds = RobotContainer.drivetrain.getChassisSpeeds();
            ChassisSpeeds robotRelativeAcceleration = RobotContainer.drivetrain.getChassisAcceleration();

            ShotCalculationResult shotResult =
                    calculateShot(shotTarget, robotPose, robotRelativeSpeeds, fuelReleaseOffset);

            Vector<N3> robotRelativeShotVector =
                    new Vector<>(robotPose.getRotation().unaryMinus().toMatrix().times(shotResult.shotVector));

            RobotContainer.turret.setTarget(calculateTurretState(
                    shotResult.shotVector, robotRelativeShotVector, robotRelativeSpeeds, robotRelativeAcceleration));

            boolean isBlocked = calculateIsHoodBlocked(
                    robotPose, robotRelativeSpeeds, RobotContainer.model.shooterModel.getShooterHoodPosition());
            Logger.recordOutput("ShootOrchestrator/IsBlocked", isBlocked);

            RobotContainer.shooter.setTargetState(
                    calculateShooterState(shotTarget, robotRelativeShotVector, isBlocked));

            boolean onTarget = isOnTarget(shotTarget, shotResult.shotCalculation(), isBlocked);
            Logger.recordOutput("ShootOrchestrator/OnTarget", onTarget);
            lastOnTarget = onTarget;

            updateFeeders(onTarget);
        }

        Logger.recordOutput(
                "ShootOrchestrator/Target",
                target.isPresent() ? new Pose3d(target.get().position, Rotation3d.kZero) : Pose3d.kZero);
        Logger.recordOutput("ShootOrchestrator/ShootingEnabled", shootingEnabled);
    }

    private void updateAlerts() {
        shootOverrideAlert.set(shootOverride.get());

        if (feedMode == null) {
            feedModeAlert.setText("Feed mode is <UNKNOWN>");
            feedModeAlert.set(true);
        } else if (feedMode != FeedMode.AUTO) {
            feedModeAlert.setText("Feed mode is " + feedMode.name());
            feedModeAlert.set(true);
        } else {
            feedModeAlert.set(false);
        }
    }

    public boolean isOnTarget() {
        return lastOnTarget;
    }

    private record ShotCalculationResult(Vector<N3> shotVector, ShotCalculation shotCalculation) {}
}
