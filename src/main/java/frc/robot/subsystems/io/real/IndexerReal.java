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
import frc.robot.subsystems.io.IndexerIO;
import frc.robot.utils.TalonFXOrchestra;

public class IndexerReal implements IndexerIO {

    protected final TalonFX indexer;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public IndexerReal() {
        indexer = new TalonFX(RobotMap.Indexer.MOTOR_ID);
        indexer.optimizeBusUtilization();

        positionSignal = indexer.getPosition();
        velocitySignal = indexer.getVelocity();
        voltageSignal = indexer.getMotorVoltage();
        currentSignal = indexer.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocitySignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(positionSignal, velocitySignal, voltageSignal, currentSignal);

        RobotContainer.orchestra.add(indexer, TalonFXOrchestra.Tracks.INDEXER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        indexer.getConfigurator().apply(config);
    }

    @Override
    public void setControl(ControlRequest request) {
        indexer.setControl(request);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
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
        indexer.close();
    }
}
