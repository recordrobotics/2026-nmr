package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.subsystems.io.SwerveModuleIOInputsAutoLogged;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;

public final class SwerveModule {

    private static final double STATIONARY_DRIVE_VELOCITY_THRESHOLD = 0.08;
    private static final double STATIONARY_TURN_VELOCITY_THRESHOLD = 1.0;
    private static final boolean USE_COSINE_COMPENSATION = false;

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private double targetDriveVelocity = 0;
    private double targetTurnPosition = 0;
    private double targetFeedforward = 0;

    private final MotionMagicVelocityVoltage driveRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicExpoVoltage turnRequest = new MotionMagicExpoVoltage(0);

    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VoltageOut turnVoltageRequest = new VoltageOut(0);

    private double lastMovementTime = Timer.getTimestamp();
    private boolean hasResetAbs = false;

    private final String name;

    private final SafeAlert absEncoderErrorAlert;
    private final SafeAlert absEncoderWarningAlert;

    private final SafeAlert driveDisconnectedAlert;
    private final SafeAlert turnDisconnectedAlert;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
     *
     * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
     *     module. Look at ModuleConstants.java for what variables are contained
     */
    public SwerveModule(String name, ModuleConstants m, SwerveModuleIO io) {
        this.io = io;
        this.name = name;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsDrive = driveConfig.Slot0;
        slot0ConfigsDrive.kS = m.driveKs();
        slot0ConfigsDrive.kV = m.driveKv();
        slot0ConfigsDrive.kA = m.driveKa();
        slot0ConfigsDrive.kP = m.driveKp();
        slot0ConfigsDrive.kI = 0;
        slot0ConfigsDrive.kD = 0;

        double wheelCircumference = m.wheelDiameter() * Math.PI;
        driveConfig.Feedback.SensorToMechanismRatio = m.driveGearRatio() / wheelCircumference;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsDrive = driveConfig.MotionMagic;
        motionMagicConfigsDrive.MotionMagicAcceleration = Constants.Swerve.DRIVE_MAX_ACCELERATION;
        motionMagicConfigsDrive.MotionMagicJerk = Constants.Swerve.DRIVE_MAX_JERK;

        io.applyDriveTalonFXConfig(driveConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(m.driveMotorSupplyCurrentLimit())
                        .withStatorCurrentLimit(m.driveMotorStatorCurrentLimit())
                        .withSupplyCurrentLowerLimit(m.driveMotorSupplyLowerCurrentLimit())
                        .withSupplyCurrentLowerTime(m.driveMotorSupplyLowerCurrentLimitTime())
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true))
                .withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsTurn = turnConfig.Slot0;
        slot0ConfigsTurn.kS = m.turnKs();
        slot0ConfigsTurn.kV = m.turnKv();
        slot0ConfigsTurn.kA = m.turnKa();
        slot0ConfigsTurn.kP = m.turnKp();
        slot0ConfigsTurn.kI = 0;
        slot0ConfigsTurn.kD = m.turnKd();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.Feedback.SensorToMechanismRatio = m.turnGearRatio();

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsTurn = turnConfig.MotionMagic;
        motionMagicConfigsTurn.MotionMagicExpo_kV = Constants.Swerve.TURN_MMEXPO_KV;
        motionMagicConfigsTurn.MotionMagicExpo_kA = Constants.Swerve.TURN_MMEXPO_KA;

        io.applyTurnTalonFXConfig(turnConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(m.turnMotorSupplyCurrentLimit())
                        .withStatorCurrentLimit(m.turnMotorStatorCurrentLimit())
                        .withSupplyCurrentLowerLimit(m.turnMotorSupplyLowerCurrentLimit())
                        .withSupplyCurrentLowerTime(m.turnMotorSupplyLowerCurrentLimitTime())
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true))
                .withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = m.turningEncoderOffset();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        io.applyTurningEncoderConfig(encoderConfig);

        // Corrects for offset in absolute wheel position
        updateInputs();

        if (inputs.encoderConnected) {
            io.setTurnMechanismPosition(inputs.encoderPositionRotations);
            inputs.turnMotorPositionRotations = inputs.encoderPositionRotations;
        }

        absEncoderErrorAlert = new SafeAlert("", AlertType.kError);
        absEncoderWarningAlert = new SafeAlert("", AlertType.kWarning);

        driveDisconnectedAlert = new SafeAlert(name + " drive motor disconnected", AlertType.kError);
        turnDisconnectedAlert = new SafeAlert(name + " turn motor disconnected", AlertType.kError);

        updateAbsEncoderAlert();

        targetTurnPosition = inputs.turnMotorPositionRotations;
    }

    private void updateAbsEncoderAlert() {
        final String prefix = name + " encoder ";

        if (inputs.encoderConnected) {
            switch (inputs.encoderMagnetHealth) {
                case Magnet_Invalid:
                    absEncoderErrorAlert.setText(prefix + "invalid magnet health");
                    absEncoderErrorAlert.set(true);
                    absEncoderWarningAlert.set(false);
                    break;
                case Magnet_Red:
                    absEncoderWarningAlert.setText(prefix + "magnet health red");
                    absEncoderWarningAlert.set(true);
                    absEncoderErrorAlert.set(false);
                    break;
                case Magnet_Orange:
                    absEncoderWarningAlert.setText(prefix + "magnet health orange");
                    absEncoderWarningAlert.set(true);
                    absEncoderErrorAlert.set(false);
                    break;
                default:
                    absEncoderWarningAlert.set(false);
                    absEncoderErrorAlert.set(false);
            }
        } else {
            absEncoderErrorAlert.setText(prefix + "disconnected");
            absEncoderErrorAlert.set(true);
            absEncoderWarningAlert.set(false);
        }
    }

    public double getTurnWheelVelocity() {
        return inputs.turnMotorVelocityRps;
    }

    /**
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                inputs.driveMotorVelocityMps, Rotation2d.fromRotations(inputs.turnMotorPositionRotations));
    }

    public SwerveModuleState getModuleStateAcceleration() {
        return new SwerveModuleState(
                inputs.driveMotorAccelerationMps2, Rotation2d.fromRotations(inputs.turnMotorPositionRotations));
    }

    /**
     *
     * @return The current position of the module as a SwerveModulePosition object.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                inputs.driveMotorPositionMeters, Rotation2d.fromRotations(inputs.turnMotorPositionRotations));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, double feedforward) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        if (!(SysIdManager.getProvider() instanceof Drivetrain.SysIdSpin)
                && !(SysIdManager.getProvider() instanceof Drivetrain.SysIdForward)) {
            desiredState.optimize(Rotation2d.fromRotations(inputs.turnMotorPositionRotations));
        }

        targetTurnPosition = desiredState.angle.getRotations();
        targetDriveVelocity = desiredState.speedMetersPerSecond;
        targetFeedforward = feedforward;
    }

    private void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module " + name, inputs);
    }

    /**
     * Checks if both the drive and turn motors are connected.
     * (encoder only matters at the start)
     * @return true if both motors are connected, false otherwise.
     */
    public boolean isConnected() {
        return inputs.driveMotorConnected && inputs.turnMotorConnected;
    }

    public void periodic() {
        updateInputs();

        updateAbsEncoderAlert();

        driveDisconnectedAlert.set(!inputs.driveMotorConnected);
        turnDisconnectedAlert.set(!inputs.turnMotorConnected);

        if (Math.abs(inputs.driveMotorVelocityMps) > STATIONARY_DRIVE_VELOCITY_THRESHOLD
                || Math.abs(inputs.turnMotorVelocityRps) > STATIONARY_TURN_VELOCITY_THRESHOLD) {
            hasResetAbs = false;
            lastMovementTime = Timer.getTimestamp();
        } else if (Timer.getTimestamp() - lastMovementTime > 2.0
                && inputs.encoderConnected
                && !hasResetAbs) { // if still for 2 seconds
            hasResetAbs = true;
            inputs.turnMotorPositionRotations = inputs.encoderPositionRotations;
            io.setTurnMechanismPosition(inputs.encoderPositionRotations);
        }

        double actualTargetDriveVelocity = targetDriveVelocity;
        if (USE_COSINE_COMPENSATION) {
            actualTargetDriveVelocity *=
                    Math.cos(Units.rotationsToRadians(targetTurnPosition - inputs.turnMotorPositionRotations));
        }

        io.setDriveControl(driveRequest.withVelocity(actualTargetDriveVelocity).withFeedForward(targetFeedforward));

        if (!(SysIdManager.getProvider() instanceof Drivetrain.SysIdTurn)) {
            io.setTurnControl(turnRequest.withPosition(targetTurnPosition));
        }
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void stop() {
        targetDriveVelocity = 0;
        io.setDriveControl(driveVoltageRequest.withOutput(0));
        io.setTurnControl(turnVoltageRequest.withOutput(0));
    }

    public void setDriveMotorVoltsSysIdOnly(double volts) {
        io.setDriveControl(driveVoltageRequest.withOutput(volts));
    }

    public double getDriveMotorVoltage() {
        return inputs.driveMotorVoltage;
    }

    public void setTurnMotorVoltsSysIdOnly(double volts) {
        io.setTurnControl(turnVoltageRequest.withOutput(volts));
    }

    public double getTurnMotorVoltage() {
        return inputs.turnMotorVoltage;
    }

    /** frees up all hardware allocations */
    public void close() {
        io.close();
    }
}
