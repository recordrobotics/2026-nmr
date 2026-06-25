package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.Constants.ClimberHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.ClimberIOInputsAutoLogged;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PositionedSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public final class Climber extends KillableSubsystem implements PoweredSubsystem, PositionedSubsystem {

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(7.5).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(5.2);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.2);

    private static final double RESET_VOLTAGE = -2.0;
    private static final double RESET_VELOCITY_THRESHOLD = 0.003;
    private static final double RESET_VELOCITY_THRESHOLD_TIME = 0.1;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final MotionMagicExpoVoltage climberRequest;
    private final VoltageOut voltageRequest;

    private ClimberHeight targetHeight = ClimberHeight.DOWN;
    private double setpoint;

    private final SysIdRoutine sysIdRoutine;

    private PositionStatus positionStatus = PositionStatus.UNKNOWN;
    private double lastMovementTime = 0;
    private boolean overrideKnown = false;

    private final SafeAlert disconnectedAlert = new SafeAlert("Climber disconnected!", AlertType.kError);

    public Climber(ClimberIO io) {
        this.io = io;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = 1.0 / Constants.Climber.METERS_PER_ROTATION;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.MAX_HEIGHT_METERS;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        Slot0Configs slot0ConfigsClimber = config.Slot0;
        slot0ConfigsClimber.kS = Constants.Climber.KS;
        slot0ConfigsClimber.kV = Constants.Climber.KV;
        slot0ConfigsClimber.kA = Constants.Climber.KA;
        slot0ConfigsClimber.kG = Constants.Climber.KG;
        slot0ConfigsClimber.kP = Constants.Climber.KP;
        slot0ConfigsClimber.kD = Constants.Climber.KD;
        slot0ConfigsClimber.GravityType = GravityTypeValue.Elevator_Static;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsClimber = config.MotionMagic;
        motionMagicConfigsClimber.MotionMagicExpo_kV = Constants.Climber.MMEXPO_KV;
        motionMagicConfigsClimber.MotionMagicExpo_kA = Constants.Climber.MMEXPO_KA;

        io.applyTalonFXConfig(config.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                        .withSupplyCurrentLowerLimit(Constants.Climber.SUPPLY_CURRENT_LOWER_LIMIT)
                        .withSupplyCurrentLowerTime(1)
                        .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true))
                .withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        climberRequest = new MotionMagicExpoVoltage(0);
        voltageRequest = new VoltageOut(0);

        setState(ClimberHeight.DOWN);

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Climber/SysIdTestState", state.toString())),
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
        Logger.processInputs("Climber", inputs);

        disconnectedAlert.set(!inputs.connected);

        RobotContainer.model.climberModel.update(inputs.positionMeters);

        if (overrideKnown) {
            positionStatus = PositionStatus.KNOWN;
        }

        if (!isForceDisabled()
                && positionStatus == PositionStatus.UNKNOWN
                && SimpleMath.isAFartherFromZeroThanB(inputs.voltage, RESET_VOLTAGE / 2)) {
            if (Math.abs(inputs.velocityMps) > RESET_VELOCITY_THRESHOLD) {
                lastMovementTime = Timer.getTimestamp();
            } else if (Timer.getTimestamp() - lastMovementTime > RESET_VELOCITY_THRESHOLD_TIME) {
                positionStatus = PositionStatus.KNOWN;
                io.setPosition(0);
                setControl();
            }
        } else {
            lastMovementTime = Timer.getTimestamp();
            if (positionStatus == PositionStatus.UNKNOWN) {
                setControl();
            }
        }

        // want to extend shotblocker but not there yet, wait for intake to retract
        if (targetHeight == ClimberHeight.UP
                && !isShotblockerExtended()
                && RobotContainer.intake.isWithinFramePerimeter()) {
            setState(targetHeight); // refresh state
        }
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public void setState(ClimberHeight height) {
        targetHeight = height;

        // guard against intake extension
        if (height != ClimberHeight.UP
                || !(RobotContainer.intake != null && !RobotContainer.intake.isWithinFramePerimeter())) {
            setpoint = height.getHeight();
        } else { // raise climber just before shotblocker extension to save time
            setpoint = Constants.Climber.CLIMBER_SAFE_SHOTBLOCKER_HEIGHT;
        }

        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysId)) {
            setControl();
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setControl(voltageRequest.withOutput(0.0));
        } else {
            setControl();
        }
    }

    private void setControl() {
        if (positionStatus == PositionStatus.UNKNOWN) {
            io.setControl(voltageRequest.withOutput(RESET_VOLTAGE).withIgnoreSoftwareLimits(true));
        } else {
            io.setControl(climberRequest.withPosition(setpoint));
        }
    }

    @AutoLogLevel(level = Level.REAL)
    public ClimberHeight getNearestHeight() {
        double currentHeight = inputs.positionMeters;

        ClimberHeight[] heights = ClimberHeight.values();
        Arrays.sort(heights, (a, b) -> Double.compare(a.getDifference(currentHeight), b.getDifference(currentHeight)));
        return heights[0];
    }

    public boolean isShotblockerExtended() {
        return inputs.positionMeters > Constants.Climber.CLIMBER_SAFE_SHOTBLOCKER_HEIGHT;
    }

    public boolean atGoal() {
        return Math.abs(setpoint - inputs.positionMeters) < Constants.Climber.AT_GOAL_POSITION_TOLERANCE
                && Math.abs(inputs.velocityMps) < Constants.Climber.AT_GOAL_VELOCITY_TOLERANCE;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void close() {
        io.close();
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.currentDraw;
    }

    @Override
    public void resetToStartPosition() {
        positionStatus = PositionStatus.UNKNOWN;
        io.setPosition(0);
    }

    @Override
    public PositionStatus getPositionStatus() {
        return positionStatus;
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.climber.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.climber.sysIdDynamic(direction);
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
