package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.TurretIO;
import frc.robot.subsystems.io.TurretIO.LimitSwitchStates;
import frc.robot.subsystems.io.TurretIOInputsAutoLogged;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PositionedSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class Turret extends KillableSubsystem implements PoweredSubsystem, PositionedSubsystem {

    public static final double MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS = Units.degreesToRotations(90);

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(2.7).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(1.0);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.0);

    private static final double RESET_VOLTAGE = 0.3;
    private static final double RESET_VELOCITY_THRESHOLD = 0.001;
    private static final double RESET_VELOCITY_THRESHOLD_TIME = 0.1;

    private static final LoggedNetworkNumber ffMul = new LoggedNetworkNumber("TURRET_FFMUL", 0.72);
    private static final LoggedNetworkNumber ffSpringLow = new LoggedNetworkNumber("TURRET_FFSPRING_LOW", 1.11);
    private static final LoggedNetworkNumber ffSpringHigh = new LoggedNetworkNumber("TURRET_FFSPRING_HIGH", 1.11);
    private static final LoggedNetworkNumber lookaheadTime = new LoggedNetworkNumber("TURRET_LOOKAHEAD", 0.1);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicExpoVoltage turretRequest = new MotionMagicExpoVoltage(
            Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS) - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private double targetPositionRotations;
    private double targetVelocityRotationsPerSecond;
    private double targetAccelerationRotationsPerSecondSquared;

    private SafeAlert disconnectedAlert = new SafeAlert("Turret disconnected!", AlertType.kError);

    private PositionStatus positionStatus = PositionStatus.UNKNOWN;

    private enum ResetState {
        NOT_RESETTING,
        SPIN_CW,
        SPIN_CCW,
        WAIT_FOR_INTAKE
    }

    private ResetState resetState = ResetState.NOT_RESETTING;
    private double lastMovementTime = 0;
    private boolean overrideKnown = false;

    public Turret(TurretIO io) {
        this.io = io;

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Turret.KS;
        slot0Configs.kV = Constants.Turret.KV;
        slot0Configs.kA = Constants.Turret.KA_MM;
        slot0Configs.kP = Constants.Turret.KP;
        slot0Configs.kD = Constants.Turret.KD;

        config.ClosedLoopGeneral.ContinuousWrap = false;

        config.MotionMagic.MotionMagicExpo_kV = Constants.Turret.MMEXPO_KV;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Turret.MMEXPO_KA;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.ROTATION_MAX_POSITION_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.ROTATION_MIN_POSITION_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Turret.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(config.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));
        setTarget(new TurretState(Constants.Turret.STARTING_POSITION_RADIANS, 0, 0));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Turret/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setControl(voltageRequest.withOutput(v)), null, this));

        PositionedSubsystemManager.getInstance().registerSubsystem(this);
    }

    @Override
    public void setOverrideKnown(boolean overrideKnown) {
        this.overrideKnown = overrideKnown;
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        disconnectedAlert.set(!inputs.connected);

        Constants.Turret.FF_MUL = ffMul.get();
        Constants.Turret.TURRET_SPRING_LOW_VOLTS = ffSpringLow.get();
        Constants.Turret.TURRET_SPRING_HIGH_VOLTS = ffSpringHigh.get();
        Constants.Turret.LOOKAHEAD_TIME = lookaheadTime.get();

        if (inputs.limitSwitchStates.hasFault()) {
            positionStatus = PositionStatus.SENSOR_FAULT;
        } else if (positionStatus == PositionStatus.SENSOR_FAULT) {
            positionStatus = PositionStatus.UNKNOWN;
        }

        if (overrideKnown) {
            positionStatus = PositionStatus.KNOWN;
        }

        setControl();

        RobotContainer.model.shooterModel.updateTurret(Units.rotationsToRadians(getPositionRotations()));

        Logger.recordOutput("Turret/TargetPositionRotations", targetPositionRotations);
        Logger.recordOutput("Turret/TargetVelocityRotationsPerSecond", targetVelocityRotationsPerSecond);
        Logger.recordOutput(
                "Turret/TargetAccelerationRotationsPerSecondSquared", targetAccelerationRotationsPerSecondSquared);

        Logger.recordOutput(
                "Turret/TargetVoltage",
                feedforward(
                        SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                        SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001)));
    }

    private void setControl() {
        if (positionStatus == PositionStatus.UNKNOWN) {
            switch (resetState) {
                case NOT_RESETTING:
                    resetState = ResetState.SPIN_CW;
                    io.setControl(voltageRequest.withOutput(-RESET_VOLTAGE));
                    break;
                case SPIN_CW:
                    checkMagnet();
                    if (isForceDisabled()
                            || (!SimpleMath.isAFartherFromZeroThanB(inputs.voltage, -RESET_VOLTAGE / 3)
                                    && !inputs.reverseSoftLimitHit)
                            || Math.abs(inputs.velocityRotationsPerSecond) > RESET_VELOCITY_THRESHOLD) {
                        lastMovementTime = Timer.getTimestamp();
                    } else if (Timer.getTimestamp() - lastMovementTime > RESET_VELOCITY_THRESHOLD_TIME) {
                        resetState = ResetState.SPIN_CCW;
                        io.setControl(voltageRequest.withOutput(RESET_VOLTAGE));
                    }
                    break;
                case SPIN_CCW:
                    checkMagnet();
                    if (isForceDisabled()
                            || (!SimpleMath.isAFartherFromZeroThanB(inputs.voltage, RESET_VOLTAGE / 3)
                                    && !inputs.forwardSoftLimitHit)
                            || Math.abs(inputs.velocityRotationsPerSecond) > RESET_VELOCITY_THRESHOLD) {
                        lastMovementTime = Timer.getTimestamp();
                    } else if (Timer.getTimestamp() - lastMovementTime > RESET_VELOCITY_THRESHOLD_TIME) {
                        if (RobotContainer.intake.isNearStartPosition()) {
                            resetState = ResetState.WAIT_FOR_INTAKE;
                        } else {
                            positionStatus = PositionStatus.RESETTING_FAULT;
                            resetState = ResetState.NOT_RESETTING;
                        }
                        io.setControl(voltageRequest.withOutput(0));
                    }
                    break;
                case WAIT_FOR_INTAKE:
                    if (!RobotContainer.intake.isNearStartPosition()) {
                        resetState = ResetState.NOT_RESETTING;
                    }
                    break;
            }
        } else if (positionStatus == PositionStatus.KNOWN
                && !isForceDisabled()
                && !(SysIdManager.getProvider() instanceof SysId)) {
            io.setControl(turretRequest
                    .withPosition(targetPositionRotations
                            - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS
                            + targetVelocityRotationsPerSecond * Constants.Turret.LOOKAHEAD_TIME)
                    .withIgnoreSoftwareLimits(false)
                    .withFeedForward(feedforward(
                            SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                            SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001))));
        }
    }

    @AutoLogLevel
    public boolean isStowed() {
        return Math.abs(Math.abs(Units.rotationsToRadians(getPositionRotations()))
                        - Math.abs(Constants.Turret.STARTING_POSITION_RADIANS))
                < Units.degreesToRadians(9);
    }

    public void checkMagnet() {
        LimitSwitchStates limitSwitchStates = inputs.limitSwitchStates;
        boolean isCcw = inputs.voltage < 0.1;
        if (limitSwitchStates.frontLeft()) {
            io.setPositionRotations(
                    isCcw
                            ? Constants.Turret.FRONT_LEFT_MAGNET_MOTOR_ROTATIONS_CCW
                            : Constants.Turret.FRONT_LEFT_MAGNET_MOTOR_ROTATIONS_CW);
            positionStatus = PositionStatus.KNOWN;
        } else if (limitSwitchStates.backLeft()) {
            io.setPositionRotations(
                    isCcw
                            ? Constants.Turret.BACK_LEFT_MAGNET_MOTOR_ROTATIONS_CCW
                            : Constants.Turret.BACK_LEFT_MAGNET_MOTOR_ROTATIONS_CW);
            positionStatus = PositionStatus.KNOWN;
        } else if (limitSwitchStates.backRight()) {
            io.setPositionRotations(
                    isCcw
                            ? Constants.Turret.BACK_RIGHT_MAGNET_MOTOR_ROTATIONS_CCW
                            : Constants.Turret.BACK_RIGHT_MAGNET_MOTOR_ROTATIONS_CW);
            positionStatus = PositionStatus.KNOWN;
        }
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getPositionRotations() {
        return inputs.positionRotations + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS;
    }

    private double getSpringFeedforward(double velocityRotationsPerSecond) {
        double pos = inputs.positionRotations;

        if (pos > Constants.Turret.TURRET_SPRING_LOW_START_POS && velocityRotationsPerSecond > 0)
            return Constants.Turret.TURRET_SPRING_LOW_VOLTS;
        if (pos > Constants.Turret.TURRET_SPRING_HIGH_START_POS && velocityRotationsPerSecond > 0)
            return Constants.Turret.TURRET_SPRING_HIGH_VOLTS;

        if (pos < Constants.Turret.TURRET_SPRING_LOW_START_NEG && velocityRotationsPerSecond < 0)
            return -Constants.Turret.TURRET_SPRING_LOW_VOLTS;
        if (pos < Constants.Turret.TURRET_SPRING_HIGH_START_NEG && velocityRotationsPerSecond < 0)
            return -Constants.Turret.TURRET_SPRING_HIGH_VOLTS;

        return 0;
    }

    private double feedforward(double velocityRotationsPerSecond, double accelerationRotationsPerSecondSquared) {
        double velocityError = velocityRotationsPerSecond - inputs.velocityRotationsPerSecond;
        return Math.max(
                        -12.0,
                        Math.min(
                                12.0,
                                Constants.Turret.KV * velocityRotationsPerSecond
                                        + Constants.Turret.KA * accelerationRotationsPerSecondSquared
                                        + Constants.Turret.KS * Math.signum(velocityRotationsPerSecond)
                                        + Constants.Turret.KVP * velocityError
                                        + getSpringFeedforward(velocityRotationsPerSecond)))
                * Constants.Turret.FF_MUL;
    }

    public static double getClosestTurretPosition(double currentPos, double targetHeading) {

        double minRange = Units.rotationsToRadians(
                Constants.Turret.ROTATION_MIN_POSITION_MOTOR_ROTATIONS + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
        double maxRange = Units.rotationsToRadians(
                Constants.Turret.ROTATION_MAX_POSITION_MOTOR_ROTATIONS + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);

        // bring target close to current revolution
        double baseTarget = targetHeading + SimpleMath.PI2 * Math.round((currentPos - targetHeading) / SimpleMath.PI2);

        double bestPos = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        // check equivalent angles (target ± 2π)
        for (int i = -1; i <= 1; i++) {
            double candidate = baseTarget + i * SimpleMath.PI2;

            if (candidate >= minRange && candidate <= maxRange) {
                double dist = Math.abs(candidate - currentPos);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestPos = candidate;
                }
            }
        }

        // if none are in range, clamp to nearest limit
        if (Double.isNaN(bestPos)) {
            double clamped = Math.max(minRange, Math.min(maxRange, baseTarget));
            bestPos = clamped;
        }

        return bestPos;
    }

    public void setTarget(TurretState state) {
        this.targetPositionRotations = Units.radiansToRotations(
                getClosestTurretPosition(Units.rotationsToRadians(getPositionRotations()), state.positionRadians()));
        this.targetVelocityRotationsPerSecond = Units.radiansToRotations(state.velocityRadiansPerSecond());
        this.targetAccelerationRotationsPerSecondSquared =
                Units.radiansToRotations(state.accelerationRadiansPerSecondSquared());
        setControl();
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setControl(voltageRequest.withOutput(0));
        } else {
            setControl();
        }
    }

    public boolean atGoal(double positionToleranceRadians) {
        return SimpleMath.isWithinTolerance(
                getPositionRotations(), targetPositionRotations, Units.radiansToRotations(positionToleranceRadians));
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.currentDraw;
    }

    public RobotToMechanismUpdate getRobotToMechanism() {
        double timestamp = Timer.getTimestamp();
        return new RobotToMechanismUpdate(
                new Transform3d(
                                Translation3d.kZero,
                                new Rotation3d(0, 0, Units.rotationsToRadians(getPositionRotations())))
                        .plus(new Transform3d(0.127, 0.127, 0, Rotation3d.kZero)),
                timestamp);
    }

    @Override
    public void resetToStartPosition() {
        positionStatus = PositionStatus.UNKNOWN;
        resetState = ResetState.NOT_RESETTING;
        io.setPositionRotations(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS)
                - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
    }

    @Override
    public PositionStatus getPositionStatus() {
        return positionStatus;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public record TurretState(
            double positionRadians, double velocityRadiansPerSecond, double accelerationRadiansPerSecondSquared) {}

    public record RobotToMechanismUpdate(Transform3d robotToMechanism, double timestamp) {}

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.turret.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.turret.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }
}
