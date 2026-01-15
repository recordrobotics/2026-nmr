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
import frc.robot.subsystems.io.SpindexerIO;
import frc.robot.utils.TalonFXOrchestra;

public class SpindexerReal implements SpindexerIO {

    protected final TalonFX spindexer;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public SpindexerReal() {
        spindexer = new TalonFX(RobotMap.Spindexer.MOTOR_ID);
        spindexer.optimizeBusUtilization();

        positionSignal = spindexer.getPosition();
        velocitySignal = spindexer.getVelocity();
        voltageSignal = spindexer.getMotorVoltage();
        currentSignal = spindexer.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocitySignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(positionSignal, velocitySignal, voltageSignal, currentSignal);

        RobotContainer.orchestra.add(spindexer, TalonFXOrchestra.Tracks.SPINDEXER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        spindexer.getConfigurator().apply(config);
    }

    @Override
    public void setControl(ControlRequest request) {
        spindexer.setControl(request);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.connected = velocitySignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRotationsPerSecond = velocitySignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.currentDraw = currentSignal.getValue();
    }

    @Override
    public void close() {
        spindexer.close();
    }
}
