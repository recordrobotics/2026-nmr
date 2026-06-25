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
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.subsystems.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.io.sim.IntakeSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PositionedSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class Intake extends KillableSubsystem implements PoweredSubsystem, PositionedSubsystem {
    // Arm Leader is on the left of the intake (right of the robot) and Arm Follower is on the right of the intake (left
    // of the robot)

    private static final double ARM_POSITION_TOLERANCE = Units.degreesToRotations(5);
    private static final double ARM_POSITION_TOLERANCE_WHEEL_START = Units.degreesToRotations(20);

    private static final double ARM_VELOCITY_TOLERANCE = Units.degreesToRotations(50);
    private static final double WHEEL_VELOCITY_TOLERANCE_MPS = 2.0; // TODO

    private static final Velocity<VoltageUnit> SYSID_ARM_RAMP_RATE =
            Volts.of(2.0).per(Second);
    private static final Voltage SYSID_ARM_STEP_VOLTAGE = Volts.of(0.8);
    private static final Time SYSID_ARM_TIMEOUT = Seconds.of(0.8);

    private static final Velocity<VoltageUnit> SYSID_WHEEL_RAMP_RATE =
            Volts.of(2.86).per(Second);
    private static final Voltage SYSID_WHEEL_STEP_VOLTAGE = Volts.of(4.3);
    private static final Time SYSID_WHEEL_TIMEOUT = Seconds.of(1.5);

    private static final double RESET_VOLTAGE = -2.0;
    private static final double RESET_VELOCITY_THRESHOLD = 0.06;
    private static final double RESET_VELOCITY_THRESHOLD_TIME = 0.1;

    // Don't use reset delta if we moved less than 40 degrees, assume ARM_STARTING_POSITION_RADIANS instead
    private static final double ENCODERS_RESET_DELTA_THRESHOLD = Units.degreesToRotations(40);

    private static final LoggedNetworkBoolean disableWheelToggle =
            new LoggedNetworkBoolean("Intake/DisableWheel", false);

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutineArm;
    private final SysIdRoutine sysIdRoutineWheel;

    private final MotionMagicExpoVoltage armRequest =
            new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS));
    private final MotionMagicVelocityVoltage wheelRequest = new MotionMagicVelocityVoltage(RotationsPerSecond.of(0.0));
    private final VoltageOut armVoltageRequest = new VoltageOut(0);
    private final VoltageOut wheelVoltageRequest = new VoltageOut(0);

    private double armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
    private double wheelTargetVelocityMps = 0.0;
    private double actualWheelTargetVelocityMps = 0.0;
    private WheelMode wheelTargetState = WheelMode.OFF;
    private IntakeState targetState = IntakeState.STARTING;

    private double lastNotJammedTime = 0;

    private PositionStatus positionStatus = PositionStatus.UNKNOWN;
    private boolean hasStartedMovingDown = false;
    private boolean runExtendHoming = false;
    private double lastMovementTime = 0;
    private boolean overrideKnown = false;

    private double lastNotNegativePositionTime = 0;

    private double encodersResetDelta = 0;

    private final SafeAlert wheelDisconnectedAlert = new SafeAlert("Intake roller disconnected!", AlertType.kError);

    public enum IntakeState {
        INTAKE,
        OUT,
        EJECT,
        STARTING,
        RETRACTED
    }

    private enum WheelMode {
        OFF,
        INTAKE,
        EJECT
    }

    public Intake(IntakeIO io) {
        this.io = io;

        TalonFXConfiguration configArm = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsLeader = configArm.Slot0;
        slot0ConfigsLeader.kS = Constants.Intake.ARM_KS;
        slot0ConfigsLeader.kV = Constants.Intake.ARM_KV;
        slot0ConfigsLeader.kA = Constants.Intake.ARM_KA;
        slot0ConfigsLeader.kG = Constants.Intake.ARM_KG;
        slot0ConfigsLeader.kP = Constants.Intake.ARM_KP;
        slot0ConfigsLeader.kD = Constants.Intake.ARM_KD;
        slot0ConfigsLeader.GravityType = GravityTypeValue.Arm_Cosine;
        slot0ConfigsLeader.GravityArmPositionOffset = Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS;

        configArm.MotionMagic.MotionMagicExpo_kV = Constants.Intake.ARM_MMEXPO_KV;
        configArm.MotionMagic.MotionMagicExpo_kA = Constants.Intake.ARM_MMEXPO_KA;

        configArm.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ARM_SUPPLY_CURRENT_LIMIT.in(Amps);
        configArm.CurrentLimits.StatorCurrentLimit = Constants.Intake.ARM_STATOR_CURRENT_LIMIT.in(Amps);
        configArm.CurrentLimits.SupplyCurrentLimitEnable = true;
        configArm.CurrentLimits.StatorCurrentLimitEnable = true;
        configArm.CurrentLimits.SupplyCurrentLowerLimit = Constants.Intake.ARM_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        configArm.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Intake.ARM_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);

        configArm.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(Constants.Intake.ARM_MAX_POSITION_RADIANS);
        configArm.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS);
        configArm.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configArm.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configArm.Feedback.SensorToMechanismRatio = Constants.Intake.ARM_GEAR_RATIO;
        configArm.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyArmTalonFXConfig(configArm.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        TalonFXConfiguration configWheel = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsWheel = configWheel.Slot0;
        slot0ConfigsWheel.kS = Constants.Intake.WHEEL_KS;
        slot0ConfigsWheel.kV = Constants.Intake.WHEEL_KV;
        slot0ConfigsWheel.kA = Constants.Intake.WHEEL_KA;
        slot0ConfigsWheel.kP = Constants.Intake.WHEEL_KP;

        configWheel.MotionMagic.MotionMagicJerk = Constants.Intake.WHEEL_MAX_JERK;
        configWheel.MotionMagic.MotionMagicAcceleration = Constants.Intake.WHEEL_MAX_ACCELERATION;

        configWheel.CurrentLimits.SupplyCurrentLimit = Constants.Intake.WHEEL_SUPPLY_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.StatorCurrentLimit = Constants.Intake.WHEEL_STATOR_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.SupplyCurrentLimitEnable = true;
        configWheel.CurrentLimits.StatorCurrentLimitEnable = true;
        configWheel.CurrentLimits.SupplyCurrentLowerLimit = Constants.Intake.WHEEL_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Intake.WHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);

        configWheel.Feedback.SensorToMechanismRatio = 1.0 / Constants.Intake.WHEEL_METERS_PER_ROTATION;
        configWheel.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configWheel.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        io.applyWheelTalonFXConfig(configWheel.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        setState(targetState);

        sysIdRoutineArm = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_ARM_RAMP_RATE,
                        SYSID_ARM_STEP_VOLTAGE,
                        SYSID_ARM_TIMEOUT,
                        state -> Logger.recordOutput("Intake/Arm/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        v -> io.setArmControl(armVoltageRequest.withOutput(
                                v.in(Volts) + 0.9 * Math.cos(Units.rotationsToRadians(inputs.armPositionRotations)))),
                        null,
                        this));

        sysIdRoutineWheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_WHEEL_RAMP_RATE, // default 1 volt/second ramp rate
                        SYSID_WHEEL_STEP_VOLTAGE, // default 7 volt step voltage
                        SYSID_WHEEL_TIMEOUT,
                        state -> Logger.recordOutput("Intake/Wheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setWheelControl(wheelVoltageRequest.withOutput(v)), null, this));

        PositionedSubsystemManager.getInstance().registerSubsystem(this);
    }

    @Override
    public void setOverrideKnown(boolean overrideKnown) {
        this.overrideKnown = overrideKnown;
    }

    public IntakeSim getSimIO() {
        if (io instanceof IntakeSim sim) {
            return sim;
        }

        return null;
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        wheelDisconnectedAlert.set(!inputs.wheelConnected);

        RobotContainer.model.intakeModel.update(Units.rotationsToRadians(inputs.armPositionRotations));

        if (!inputs.armHasPosition) {
            positionStatus = PositionStatus.UNKNOWN;
            encodersResetDelta = 0;
        }

        if (overrideKnown) {
            positionStatus = PositionStatus.KNOWN;
        }

        if (!isForceDisabled()
                && (targetState == IntakeState.INTAKE
                        || targetState == IntakeState.EJECT
                        || targetState == IntakeState.OUT)
                && positionStatus == PositionStatus.UNKNOWN) {
            if (runExtendHoming) {
                if (SimpleMath.isAFartherFromZeroThanB(inputs.armVoltage, RESET_VOLTAGE / 3)) {
                    if (Math.abs(inputs.armVelocityRotationsPerSecond) > RESET_VELOCITY_THRESHOLD) {
                        lastMovementTime = Timer.getTimestamp();
                    } else if (Timer.getTimestamp() - lastMovementTime > RESET_VELOCITY_THRESHOLD_TIME) {
                        positionStatus = PositionStatus.KNOWN;
                        runExtendHoming = false;
                        hasStartedMovingDown = false;
                        encodersResetDelta = inputs.armPositionRotations
                                - Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
                        io.setArmPositionRotations(
                                Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS));
                        setArmControl();
                    }
                } else {
                    lastMovementTime = Timer.getTimestamp();
                }
            } else {
                if (inputs.armVoltage > -1.0 && hasStartedMovingDown) {
                    runExtendHoming = true;
                    lastMovementTime = Timer.getTimestamp();
                    setArmControl();
                } else if (inputs.armVoltage < -1.0) {
                    hasStartedMovingDown = true;
                }
            }
        }

        if (!isForceDisabled() && positionStatus == PositionStatus.KNOWN) {
            if (inputs.armPositionRotations > Units.degreesToRotations(-1.5)) {
                lastNotNegativePositionTime = Timer.getTimestamp();
            } else if (Timer.getTimestamp() - lastNotNegativePositionTime > 0.9) {
                lastNotNegativePositionTime = Timer.getTimestamp();
                io.setArmPositionRotations(Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS));
            }
        }

        if (
        // want to go to starting config but not there yet, wait for turret to stow
        (targetState == IntakeState.STARTING && !isNearStartPosition() && RobotContainer.turret.isStowed())
                ||
                // want to extend but not there yet, wait for shotblocker to go down
                (targetState != IntakeState.STARTING
                        && isNearStartPosition()
                        && !RobotContainer.climber.isShotblockerExtended())) {
            setState(targetState); // refresh state
        }

        if (!isForceDisabled()
                && !runExtendHoming
                && !(SysIdManager.getProvider() instanceof SysIdArm)
                && !(SysIdManager.getProvider() instanceof Turret.SysId)) setArmControl();
        if (!(SysIdManager.getProvider() instanceof SysIdWheel)) setWheelControl();

        Logger.recordOutput("Intake/EncoderResetDelta", encodersResetDelta);
    }

    private void setArmTarget(IntakeState state) {
        if (state == IntakeState.STARTING
                && !(
                /* not near start and turret isn't stowed (need to stow turret first) */
                !isNearStartPosition() && RobotContainer.turret != null && !RobotContainer.turret.isStowed())) {
            armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_UP_POSITION_RADIANS);
        } else if (!(RobotContainer.climber != null
                && RobotContainer.climber.isShotblockerExtended())) { // guard against shotblocker extension
            switch (state) {
                case INTAKE, OUT:
                    armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS);
                    break;
                case EJECT:
                    armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_EJECT_POSITION_RADIANS);
                    break;
                case RETRACTED:
                    armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_RETRACTED_POSITION_RADIANS);
                    break;
            }
        }
    }

    public void setState(IntakeState state) {
        targetState = state;

        setArmTarget(state);

        wheelTargetState = switch (state) {
            case INTAKE, RETRACTED -> WheelMode.INTAKE;
            case EJECT -> WheelMode.EJECT;
            case STARTING, OUT -> WheelMode.OFF;
        };

        if (state == IntakeState.STARTING || state == IntakeState.RETRACTED) {
            runExtendHoming = false;
            hasStartedMovingDown = false;
        }

        if (!isForceDisabled()
                && !runExtendHoming
                && !(SysIdManager.getProvider() instanceof SysIdArm)
                && !(SysIdManager.getProvider() instanceof Turret.SysId)) setArmControl();

        if (!(SysIdManager.getProvider() instanceof SysIdWheel)) setWheelControl();
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setArmControl(armVoltageRequest.withOutput(0.0));
            io.setWheelControl(wheelVoltageRequest.withOutput(0.0));
        } else {
            setArmControl();
            setWheelControl();
        }
    }

    private void setArmControl() {
        if (isForceDisabled()) {
            io.setArmControl(armVoltageRequest.withOutput(0.0));
        } else if (runExtendHoming && positionStatus == PositionStatus.UNKNOWN) {
            io.setArmControl(armVoltageRequest.withOutput(RESET_VOLTAGE).withIgnoreSoftwareLimits(true));
        } else {
            double feedforward = 0;
            if ((targetState == IntakeState.INTAKE || targetState == IntakeState.OUT) && !isNearStartPosition()) {
                feedforward = Constants.Intake.ARM_DOWN_FF;
            } else if (targetState == IntakeState.STARTING
                    && inputs.armPositionRotations < Units.degreesToRotations(127)) {
                feedforward = Constants.Intake.ARM_UP_FF;
            } else if (targetState == IntakeState.STARTING) {
                feedforward = Constants.Intake.ARM_UP_HOLD_FF;
            }
            io.setArmControl(armRequest
                    .withPosition(armTargetRotations)
                    .withLimitForwardMotion((targetState == IntakeState.INTAKE || targetState == IntakeState.OUT)
                            && !isNearStartPosition())
                    .withFeedForward(feedforward)); // follower will follow this
        }
    }

    private void setWheelControl() {

        switch (wheelTargetState) {
            case OFF:
                wheelTargetVelocityMps = 0;
                break;
            case EJECT:
                wheelTargetVelocityMps = Constants.Intake.WHEEL_EJECT_VELOCITY_MPS;
                break;
            case INTAKE:
                if (actualWheelTargetVelocityMps > 1) {
                    if (inputs.wheelVelocityMps >= 1) {
                        lastNotJammedTime = Timer.getTimestamp();
                    } else if (Timer.getTimestamp() - lastNotJammedTime >= 1.0) {
                        wheelTargetVelocityMps = Constants.Intake.WHEEL_JAMMED_VELOCITY_MPS;
                    } else {
                        wheelTargetVelocityMps = Constants.Intake.WHEEL_INTAKE_VELOCITY_MPS;
                    }
                } else {
                    wheelTargetVelocityMps = Constants.Intake.WHEEL_INTAKE_VELOCITY_MPS;
                }
                break;
        }

        boolean armNearGoal = SimpleMath.isWithinTolerance(
                inputs.armPositionRotations, armTargetRotations, ARM_POSITION_TOLERANCE_WHEEL_START);

        if (armTargetRotations >= inputs.armPositionRotations /* update wheels immediately if raising intake */
                || armNearGoal
                || Math.abs(wheelTargetVelocityMps) < Math.abs(actualWheelTargetVelocityMps)) {
            actualWheelTargetVelocityMps = wheelTargetVelocityMps;

            if (!isForceDisabled()) {
                if (disableWheelToggle.get()) {
                    io.setWheelControl(wheelRequest.withVelocity(0));
                } else if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysIdWheel)) {
                    io.setWheelControl(wheelRequest.withVelocity(actualWheelTargetVelocityMps));
                }
            }
        }
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public IntakeState getTargetState() {
        return targetState;
    }

    public boolean isNearStartPosition() {
        return inputs.armPositionRotations >= getArmStartPosition() - Units.degreesToRotations(42);
    }

    public boolean isWithinFramePerimeter() {
        return inputs.armPositionRotations >= getArmStartPosition() - Units.degreesToRotations(10);
    }

    private double getArmStartPosition() {
        // if (overrideKnown || Math.abs(encodersResetDelta) < ENCODERS_RESET_DELTA_THRESHOLD) {
        //     return Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
        // } else {
        //     return Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS)
        //             - encodersResetDelta; // hardstop is at down position, delta is relative to that
        // }
        return Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
    }

    private boolean armAtGoal() {
        return SimpleMath.isWithinTolerance(inputs.armPositionRotations, armTargetRotations, ARM_POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(inputs.armVelocityRotationsPerSecond, 0, ARM_VELOCITY_TOLERANCE);
    }

    private boolean wheelAtGoal() {
        return SimpleMath.isWithinTolerance(
                inputs.wheelVelocityMps, wheelTargetVelocityMps, WHEEL_VELOCITY_TOLERANCE_MPS);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public boolean atGoal() {
        return armAtGoal() && wheelAtGoal();
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.armCurrentDraw.plus(inputs.wheelCurrentDraw);
    }

    @Override
    public void resetToStartPosition() {
        positionStatus = PositionStatus.UNKNOWN;
        hasStartedMovingDown = false;
        runExtendHoming = false;
        encodersResetDelta = 0;
        io.setArmPositionRotations(Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS));
    }

    @Override
    public PositionStatus getPositionStatus() {
        return positionStatus;
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public Command sysIdQuasistaticWheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineWheel.quasistatic(direction);
    }

    public Command sysIdDynamicWheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineWheel.dynamic(direction);
    }

    public Command sysIdQuasistaticArm(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.quasistatic(direction);
    }

    public Command sysIdDynamicArm(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.dynamic(direction);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public static class SysIdArm implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.intake.sysIdQuasistaticArm(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.intake.sysIdDynamicArm(direction);
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

    public static class SysIdWheel implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.intake.sysIdQuasistaticWheel(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.intake.sysIdDynamicWheel(direction);
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
