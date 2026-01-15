package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
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
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.subsystems.io.ShooterIOInputsAutoLogged;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PositionedSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends KillableSubsystem implements PoweredSubsystem, PositionedSubsystem {

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(1.3).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(0.4);
    private static final Time SYSID_TIMEOUT = Seconds.of(0.9);

    private static final double RESET_VOLTAGE = 1.0;
    private static final double RESET_VELOCITY_THRESHOLD = 0.01;
    private static final double RESET_VELOCITY_THRESHOLD_TIME = 0.1;

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutineFlywheel;
    private final SysIdRoutine sysIdRoutineHood;

    private final MotionMagicExpoVoltage hoodRequest =
            new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
    private final MotionMagicVelocityVoltage flywheelRequest = new MotionMagicVelocityVoltage(0.0);
    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);

    private final SafeAlert hoodDisconnectedAlert = new SafeAlert("Hood disconnected!", AlertType.kError);

    private final ExponentialProfile hoodProfile =
            new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
                    7.0, Constants.Shooter.HOOD_MMEXPO_KV, Constants.Shooter.HOOD_MMEXPO_KA));

    private double hoodTargetPositionRotations;
    private double flywheelTargetVelocityMps;
    private double flywheelFeedforward;

    private PositionStatus positionStatus = PositionStatus.UNKNOWN;
    private double lastMovementTime = 0;
    private boolean overrideKnown = false;

    private boolean handledRotorFault = false;

    public Shooter(ShooterIO io) {
        this.io = io;

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        Slot0Configs hoodSlot0Configs = hoodConfig.Slot0;
        hoodSlot0Configs.kS = Constants.Shooter.HOOD_KS;
        hoodSlot0Configs.kV = Constants.Shooter.HOOD_KV;
        hoodSlot0Configs.kA = Constants.Shooter.HOOD_KA;
        hoodSlot0Configs.kP = Constants.Shooter.HOOD_KP;
        hoodSlot0Configs.kD = Constants.Shooter.HOOD_KD;
        hoodSlot0Configs.kG = Constants.Shooter.HOOD_KG;
        hoodSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        hoodSlot0Configs.GravityArmPositionOffset = Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS;

        hoodConfig.MotionMagic.MotionMagicExpo_kV = Constants.Shooter.HOOD_MMEXPO_KV;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = Constants.Shooter.HOOD_MMEXPO_KA;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.HOOD_SUPPLY_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Shooter.HOOD_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Shooter.HOOD_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.HOOD_STATOR_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(Constants.Shooter.HOOD_MAX_POSITION_RADIANS);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(Constants.Shooter.HOOD_MIN_POSITION_RADIANS);
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.HOOD_GEAR_RATIO;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        io.applyHoodTalonFXConfig(hoodConfig.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        Slot0Configs flywheelSlot0Configs = flywheelConfig.Slot0;
        flywheelSlot0Configs.kS = Constants.Shooter.FLYWHEEL_KS;
        flywheelSlot0Configs.kV = Constants.Shooter.FLYWHEEL_KV;
        flywheelSlot0Configs.kA = Constants.Shooter.FLYWHEEL_KA;
        flywheelSlot0Configs.kP = Constants.Shooter.FLYWHEEL_KP;

        flywheelConfig.MotionMagic.MotionMagicJerk = Constants.Shooter.FLYWHEEL_MAX_JERK;
        flywheelConfig.MotionMagic.MotionMagicAcceleration = Constants.Shooter.FLYWHEEL_MAX_ACCELERATION;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.FLYWHEEL_SUPPLY_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit =
                Constants.Shooter.FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Shooter.FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.FLYWHEEL_STATOR_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        flywheelConfig.Feedback.SensorToMechanismRatio = 1.0 / Constants.Shooter.FLYWHEEL_METERS_PER_ROTATION;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        io.applyFlywheelTalonFXConfig(flywheelConfig.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        setTargetState(new ShooterState(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS, 0.0, 0));

        sysIdRoutineFlywheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("Shooter/Flywheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        v -> io.setFlywheelControl(flywheelVoltageRequest.withOutput(v)), null, this));

        sysIdRoutineHood = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Shooter/Hood/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setHoodControl(hoodVoltageRequest.withOutput(v)), null, this));

        PositionedSubsystemManager.getInstance().registerSubsystem(this);
    }

    @Override
    public void setOverrideKnown(boolean overrideKnown) {
        this.overrideKnown = overrideKnown;
    }

    public void setTargetState(ShooterState targetState) {
        hoodTargetPositionRotations = Units.radiansToRotations(targetState.hoodAngleRadians);
        flywheelTargetVelocityMps = targetState.flywheelVelocityMps;
        flywheelFeedforward = targetState.feedforward;

        if (!isForceDisabled()) {
            if (!(SysIdManager.getProvider() instanceof SysIdHood)) setHoodControl();
            if (!(SysIdManager.getProvider() instanceof SysIdFlywheel))
                io.setFlywheelControl(
                        flywheelRequest.withVelocity(flywheelTargetVelocityMps).withFeedForward(flywheelFeedforward));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setHoodControl(hoodVoltageRequest.withOutput(0.0));
            io.setFlywheelControl(flywheelVoltageRequest.withOutput(0.0));
        } else {
            setHoodControl();
            io.setFlywheelControl(
                    flywheelRequest.withVelocity(flywheelTargetVelocityMps).withFeedForward(flywheelFeedforward));
        }
    }

    private void setHoodControl() {
        if (positionStatus == PositionStatus.UNKNOWN) {
            io.setHoodControl(hoodVoltageRequest.withOutput(RESET_VOLTAGE).withIgnoreSoftwareLimits(true));
        } else {
            io.setHoodControl(hoodRequest.withPosition(hoodTargetPositionRotations));
        }
    }

    private void checkHoodBeyondLimit() {
        if (getHoodAngle() < Constants.Shooter.HOOD_MIN_POSITION_RADIANS) {
            positionStatus = PositionStatus.UNKNOWN;
        }
    }

    public double getFlywheelVelocityMps() {
        return inputs.flywheelVelocityMps;
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        hoodDisconnectedAlert.set(!inputs.hoodConnected);

        RobotContainer.model.shooterModel.updateHood(getHoodAngle());

        checkHoodBeyondLimit();

        if (overrideKnown) {
            positionStatus = PositionStatus.KNOWN;
        }

        if (inputs.hoodRotorFault && !handledRotorFault) {
            handledRotorFault = true;
            positionStatus = PositionStatus.UNKNOWN;
        } else if (!inputs.hoodRotorFault) {
            handledRotorFault = false;
        }

        if (!isForceDisabled()
                && positionStatus == PositionStatus.UNKNOWN
                && SimpleMath.isAFartherFromZeroThanB(inputs.hoodVoltage, RESET_VOLTAGE / 2)) {
            if (Math.abs(inputs.hoodVelocityRotationsPerSecond) > RESET_VELOCITY_THRESHOLD) {
                lastMovementTime = Timer.getTimestamp();
            } else if (Timer.getTimestamp() - lastMovementTime > RESET_VELOCITY_THRESHOLD_TIME) {
                positionStatus = PositionStatus.KNOWN;
                io.setHoodPositionRotations(Units.radiansToRotations(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
                setHoodControl();
            }
        } else {
            lastMovementTime = Timer.getTimestamp();
        }
    }

    public boolean isAtTargetState(double angleToleranceRadians, double velocityToleranceMps) {
        return SimpleMath.isWithinTolerance(
                        inputs.hoodPositionRotations,
                        hoodTargetPositionRotations,
                        Units.radiansToRotations(angleToleranceRadians))
                && SimpleMath.isWithinTolerance(
                        inputs.flywheelVelocityMps, flywheelTargetVelocityMps, velocityToleranceMps);
    }

    public boolean isAtTargetState(
            double angleMinRadians, double angleMaxRadians, double velocityMinMps, double velocityMaxMps) {
        return inputs.hoodPositionRotations >= Units.radiansToRotations(angleMinRadians) + hoodTargetPositionRotations
                && inputs.hoodPositionRotations
                        <= Units.radiansToRotations(angleMaxRadians) + hoodTargetPositionRotations
                && inputs.flywheelVelocityMps >= velocityMinMps + flywheelTargetVelocityMps
                && inputs.flywheelVelocityMps <= velocityMaxMps + flywheelTargetVelocityMps;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getTargetHoodAngle() {
        return Units.rotationsToRadians(hoodTargetPositionRotations);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getTargetFlywheelVelocityMps() {
        return flywheelTargetVelocityMps;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getHoodAngle() {
        return Units.rotationsToRadians(inputs.hoodPositionRotations);
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.flywheelCurrentDraw.plus(inputs.hoodCurrentDraw);
    }

    public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.quasistatic(direction);
    }

    public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.dynamic(direction);
    }

    public Command sysIdQuasistaticHood(SysIdRoutine.Direction direction) {
        return sysIdRoutineHood.quasistatic(direction);
    }

    public Command sysIdDynamicHood(SysIdRoutine.Direction direction) {
        return sysIdRoutineHood.dynamic(direction);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    @Override
    public void resetToStartPosition() {
        positionStatus = PositionStatus.UNKNOWN;
        io.setHoodPositionRotations(Units.radiansToRotations(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
    }

    @Override
    public PositionStatus getPositionStatus() {
        return positionStatus;
    }

    public double getTimeUntilHoodAt(double targetHoodAngle) {
        State currentState = new State(inputs.hoodPositionRotations, inputs.hoodVelocityRotationsPerSecond);
        double targetPositionRotations = Units.radiansToRotations(targetHoodAngle);
        return hoodProfile.timeLeftUntil(currentState, new State(targetPositionRotations, 0));
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public record ShooterState(double hoodAngleRadians, double flywheelVelocityMps, double feedforward) {}

    public static class SysIdHood implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.shooter.sysIdQuasistaticHood(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.shooter.sysIdDynamicHood(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return true;
        }
    }

    public static class SysIdFlywheel implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.shooter.sysIdQuasistaticFlywheel(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.shooter.sysIdDynamicFlywheel(direction);
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
