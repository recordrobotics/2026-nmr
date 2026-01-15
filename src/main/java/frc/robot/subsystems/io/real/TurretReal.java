package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.TurretIO;
import frc.robot.utils.TalonFXOrchestra;

public class TurretReal implements TurretIO {

    protected final TalonFX turret;

    protected final DigitalInput frontLeftLimitSwitch = new DigitalInput(RobotMap.Turret.FRONT_LEFT_LIMIT_SWITCH_ID);
    protected final DigitalInput backLeftLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_LEFT_LIMIT_SWITCH_ID);
    protected final DigitalInput backRightLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_RIGHT_LIMIT_SWITCH_ID);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    private final StatusSignal<Boolean> forwardSoftLimitSignal;
    private final StatusSignal<Boolean> reverseSoftLimitSignal;

    public TurretReal() {
        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
        turret.optimizeBusUtilization();

        positionSignal = turret.getPosition();
        velocitySignal = turret.getVelocity();
        voltageSignal = turret.getMotorVoltage();
        currentSignal = turret.getSupplyCurrent();

        forwardSoftLimitSignal = turret.getFault_ForwardSoftLimit();
        reverseSoftLimitSignal = turret.getFault_ReverseSoftLimit();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50), positionSignal, velocitySignal, forwardSoftLimitSignal, reverseSoftLimitSignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(
                positionSignal,
                velocitySignal,
                voltageSignal,
                currentSignal,
                forwardSoftLimitSignal,
                reverseSoftLimitSignal);

        RobotContainer.orchestra.add(turret, TalonFXOrchestra.Tracks.TURRET);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        turret.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        turret.setControl(request);
    }

    @Override
    public void setPositionRotations(double newValue) {
        turret.setPosition(newValue);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = positionSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRotationsPerSecond = velocitySignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.currentDraw = currentSignal.getValue();

        inputs.forwardSoftLimitHit = forwardSoftLimitSignal.getValue().booleanValue();
        inputs.reverseSoftLimitHit = reverseSoftLimitSignal.getValue().booleanValue();

        inputs.limitSwitchStates = new LimitSwitchStates(
                !frontLeftLimitSwitch.get(), !backLeftLimitSwitch.get(), !backRightLimitSwitch.get());
    }

    @Override
    public void close() {
        turret.close();
        frontLeftLimitSwitch.close();
        backLeftLimitSwitch.close();
        backRightLimitSwitch.close();
    }
}
