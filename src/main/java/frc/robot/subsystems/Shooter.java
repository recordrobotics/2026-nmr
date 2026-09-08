package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.subsystems.io.ShooterIOInputsAutoLogged;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends KillableSubsystem implements PoweredSubsystem {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutineFlywheel;

    private final MotionMagicVelocityVoltage flywheelRequest = new MotionMagicVelocityVoltage(0.0);
    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
    private double flywheelTargetVelocityMps;
    private double flywheelFeedforward;

    public Shooter(ShooterIO io) {
        this.io = io;

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

        setTargetState(new ShooterState(0.0, 0));

        sysIdRoutineFlywheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("Shooter/Flywheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        v -> io.setFlywheelControl(flywheelVoltageRequest.withOutput(v)), null, this));
    }

    public void setTargetState(ShooterState targetState) {
        flywheelTargetVelocityMps = targetState.flywheelVelocityMps;
        flywheelFeedforward = targetState.feedforward;

        if (!isForceDisabled()) {
            if (!(SysIdManager.getProvider() instanceof SysIdFlywheel))
                io.setFlywheelControl(
                        flywheelRequest.withVelocity(flywheelTargetVelocityMps).withFeedForward(flywheelFeedforward));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setFlywheelControl(flywheelVoltageRequest.withOutput(0.0));
        } else {
            io.setFlywheelControl(
                    flywheelRequest.withVelocity(flywheelTargetVelocityMps).withFeedForward(flywheelFeedforward));
        }
    }

    public double getFlywheelVelocityMps() {
        return inputs.flywheelVelocityMps;
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public boolean isAtTargetState(double velocityToleranceMps) {
        return SimpleMath.isWithinTolerance(
                inputs.flywheelVelocityMps, flywheelTargetVelocityMps, velocityToleranceMps);
    }

    public boolean isAtTargetState(double velocityMinMps, double velocityMaxMps) {
        return inputs.flywheelVelocityMps >= velocityMinMps + flywheelTargetVelocityMps
                && inputs.flywheelVelocityMps <= velocityMaxMps + flywheelTargetVelocityMps;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getTargetFlywheelVelocityMps() {
        return flywheelTargetVelocityMps;
    }

    @Override
    public Current getCurrentDraw() {
        return inputs.flywheelCurrentDraw;
    }

    public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.quasistatic(direction);
    }

    public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.dynamic(direction);
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

    public record ShooterState(double flywheelVelocityMps, double feedforward) {}

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
