package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleReal implements SwerveModuleIO {

    protected final TalonFX driveMotor;
    protected final TalonFX turningMotor;
    protected final CANcoder absoluteTurningMotorEncoder;

    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<AngularVelocity> driveVelocitySignal;
    private final StatusSignal<AngularAcceleration> driveAccelerationSignal;
    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Current> driveCurrentSignal;

    private final StatusSignal<Angle> turnPositionSignal;
    private final StatusSignal<AngularVelocity> turnVelocitySignal;
    private final StatusSignal<Voltage> turnVoltageSignal;
    private final StatusSignal<Current> turnCurrentSignal;

    private final StatusSignal<Angle> encoderPositionSignal;
    private final StatusSignal<MagnetHealthValue> encoderMagnetHealthSignal;

    public SwerveModuleReal(ModuleConstants m, int driveTrack, int turnTrack) {
        driveMotor = new TalonFX(m.driveMotorChannel());
        turningMotor = new TalonFX(m.turningMotorChannel());
        absoluteTurningMotorEncoder = new CANcoder(m.absoluteTurningMotorEncoderChannel());

        ParentDevice.optimizeBusUtilizationForAll(driveMotor, turningMotor, absoluteTurningMotorEncoder);

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAccelerationSignal = driveMotor.getAcceleration();
        driveVoltageSignal = driveMotor.getMotorVoltage();
        driveCurrentSignal = driveMotor.getSupplyCurrent();

        turnPositionSignal = turningMotor.getPosition();
        turnVelocitySignal = turningMotor.getVelocity();
        turnVoltageSignal = turningMotor.getMotorVoltage();
        turnCurrentSignal = turningMotor.getSupplyCurrent();

        encoderPositionSignal = absoluteTurningMotorEncoder.getAbsolutePosition();
        encoderMagnetHealthSignal = absoluteTurningMotorEncoder.getMagnetHealth();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50),
                drivePositionSignal,
                driveVelocitySignal,
                driveAccelerationSignal,
                turnPositionSignal,
                turnVelocitySignal,
                encoderPositionSignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(
                drivePositionSignal,
                driveVelocitySignal,
                driveAccelerationSignal,
                driveVoltageSignal,
                driveCurrentSignal,
                turnPositionSignal,
                turnVelocitySignal,
                turnVoltageSignal,
                turnCurrentSignal,
                encoderPositionSignal,
                encoderMagnetHealthSignal);

        RobotContainer.orchestra.add(driveMotor, driveTrack);
        RobotContainer.orchestra.add(turningMotor, turnTrack);
    }

    @Override
    public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {
        driveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {
        turningMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurningEncoderConfig(CANcoderConfiguration configuration) {
        absoluteTurningMotorEncoder.getConfigurator().apply(configuration);
    }

    @Override
    public void setDriveControl(ControlRequest request) {
        driveMotor.setControl(request);
    }

    @Override
    public void setTurnControl(ControlRequest request) {
        turningMotor.setControl(request);
    }

    @Override
    public void setDriveMechanismPosition(double newValue) {
        driveMotor.setPosition(newValue);
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        turningMotor.setPosition(newValue);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorConnected = driveVelocitySignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.driveMotorPositionMeters = BaseStatusSignal.getLatencyCompensatedValue(
                        drivePositionSignal, driveVelocitySignal)
                .magnitude();
        inputs.driveMotorVelocityMps = driveVelocitySignal.getValueAsDouble();
        inputs.driveMotorAccelerationMps2 = driveAccelerationSignal.getValueAsDouble();
        inputs.driveMotorVoltage = driveVoltageSignal.getValueAsDouble();
        inputs.driveMotorCurrentDraw = driveCurrentSignal.getValue();

        inputs.turnMotorConnected = turnPositionSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.turnMotorPositionRotations = BaseStatusSignal.getLatencyCompensatedValue(
                        turnPositionSignal, turnVelocitySignal)
                .magnitude();
        inputs.turnMotorVelocityRps = turnVelocitySignal.getValueAsDouble();
        inputs.turnMotorVoltage = turnVoltageSignal.getValueAsDouble();
        inputs.turnMotorCurrentDraw = turnCurrentSignal.getValue();

        inputs.encoderConnected = encoderPositionSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.encoderPositionRotations = encoderPositionSignal.getValueAsDouble();
        inputs.encoderMagnetHealth = encoderMagnetHealthSignal.getValue();
    }

    @Override
    public void close() {
        driveMotor.close();
        turningMotor.close();
        absoluteTurningMotorEncoder.close();
    }
}
