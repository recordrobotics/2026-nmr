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
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.utils.TalonFXOrchestra;

public class ClimberReal implements ClimberIO {

    protected final TalonFX motor;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public ClimberReal() {
        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
        motor.optimizeBusUtilization();

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), positionSignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(positionSignal, velocitySignal, voltageSignal, currentSignal);

        RobotContainer.orchestra.add(motor, TalonFXOrchestra.Tracks.CLIMBER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    @Override
    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.connected = positionSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.positionMeters = positionSignal.getValueAsDouble();
        inputs.velocityMps = velocitySignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.currentDraw = currentSignal.getValue();
    }

    @Override
    public void close() {
        motor.close();
    }
}
