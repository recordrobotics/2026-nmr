package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.IndexerIO;
import frc.robot.subsystems.io.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.io.sim.IndexerSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;

public final class Indexer extends KillableSubsystem implements PoweredSubsystem {

    private static final double VELOCITY_TOLERANCE_RPS = 15.0; // TODO

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final SafeAlert disconnectedAlert = new SafeAlert("Indexer disconnected!", AlertType.kError);

    private double targetVelocityRps;
    private IndexerState targetState = IndexerState.OFF;

    public enum IndexerState {
        OFF,
        ON,
        UNSTUCK
    }

    public Indexer(IndexerIO io) {
        this.io = io;

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Indexer.KS;
        slot0Configs.kV = Constants.Indexer.KV;
        slot0Configs.kA = Constants.Indexer.KA;
        slot0Configs.kP = Constants.Indexer.KP;

        config.MotionMagic.MotionMagicJerk = Constants.Indexer.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration = Constants.Indexer.MAX_ACCELERATION;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Indexer.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Indexer.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Indexer.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Indexer.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        io.applyTalonFXConfig(config.withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        setState(IndexerState.OFF);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("Indexer/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setControl(voltageRequest.withOutput(v)), null, this));
    }

    public IndexerSim getSimIO() {
        if (io instanceof IndexerSim sim) {
            return sim;
        }

        return null;
    }

    public void setState(IndexerState newState) {
        targetState = newState;

        targetVelocityRps = switch (targetState) {
            case OFF -> 0.0;
            case ON -> Constants.Indexer.INTAKE_VELOCITY_RPS;
            case UNSTUCK -> Constants.Indexer.UNSTUCK_VELOCITY_RPS;
        };

        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysId)) {
            io.setControl(request.withVelocity(targetVelocityRps));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setControl(voltageRequest.withOutput(0.0));
        } else {
            io.setControl(request.withVelocity(targetVelocityRps));
        }
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public IndexerState getTargetState() {
        return targetState;
    }

    public boolean atGoal() {
        return SimpleMath.isWithinTolerance(
                inputs.velocityRotationsPerSecond, targetVelocityRps, VELOCITY_TOLERANCE_RPS);
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.currentDraw;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        disconnectedAlert.set(!inputs.connected);
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

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.indexer.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.indexer.sysIdDynamic(direction);
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
