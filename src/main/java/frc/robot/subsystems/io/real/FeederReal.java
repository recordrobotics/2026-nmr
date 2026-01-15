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
import frc.robot.subsystems.io.FeederIO;
import frc.robot.utils.TalonFXOrchestra;

public class FeederReal implements FeederIO {

    protected final TalonFX feeder;
    protected final DigitalInput bottomBeambreak = new DigitalInput(RobotMap.Feeder.BOTTOM_BEAM_BREAK_ID);
    protected final DigitalInput topBeambreak = new DigitalInput(RobotMap.Feeder.TOP_BEAM_BREAK_ID);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public FeederReal() {
        feeder = new TalonFX(RobotMap.Feeder.MOTOR_ID);
        feeder.optimizeBusUtilization();

        positionSignal = feeder.getPosition();
        velocitySignal = feeder.getVelocity();
        voltageSignal = feeder.getMotorVoltage();
        currentSignal = feeder.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocitySignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(positionSignal, velocitySignal, voltageSignal, currentSignal);

        RobotContainer.orchestra.add(feeder, TalonFXOrchestra.Tracks.FEEDER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        feeder.getConfigurator().apply(config);
    }

    @Override
    public void setControl(ControlRequest request) {
        feeder.setControl(request);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.connected = velocitySignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRotationsPerSecond = velocitySignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.currentDraw = currentSignal.getValue();

        inputs.bottomBeamBroken = !bottomBeambreak.get();
        inputs.topBeamBroken = !topBeambreak.get();
    }

    @Override
    public void close() {
        feeder.close();
        bottomBeambreak.close();
        topBeambreak.close();
    }
}
