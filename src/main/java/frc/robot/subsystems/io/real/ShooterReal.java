package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import frc.robot.utils.TalonFXOrchestra;
import java.util.Arrays;

public class ShooterReal implements ShooterIO {

    protected final TalonFXMotorGroup flywheelGroup;
    protected final TalonFX hood;

    private final StatusSignal<Angle> hoodPositionSignal;
    private final StatusSignal<AngularVelocity> hoodVelocitySignal;
    private final StatusSignal<Voltage> hoodVoltageSignal;
    private final StatusSignal<Current> hoodCurrentSignal;

    private final StatusSignal<Boolean> hoodRotorFault1Signal;
    private final StatusSignal<Boolean> hoodRotorFault2Signal;

    public ShooterReal() {
        flywheelGroup = new TalonFXMotorGroup(
                "Shooter",
                false,
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));
        hood = new TalonFX(RobotMap.Shooter.HOOD_ID);
        hood.optimizeBusUtilization();

        hoodPositionSignal = hood.getPosition();
        hoodVelocitySignal = hood.getVelocity();
        hoodVoltageSignal = hood.getMotorVoltage();
        hoodCurrentSignal = hood.getSupplyCurrent();
        hoodRotorFault1Signal = hood.getFault_RotorFault1();
        hoodRotorFault2Signal = hood.getFault_RotorFault2();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), hoodPositionSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), flywheelGroup.getAllHighRefreshRateStatusSignals());

        RobotContainer.allStatusSignalsToRefresh.addAll(
                hoodPositionSignal,
                hoodVelocitySignal,
                hoodVoltageSignal,
                hoodCurrentSignal,
                hoodRotorFault1Signal,
                hoodRotorFault2Signal);
        RobotContainer.allStatusSignalsToRefresh.addAll(flywheelGroup.getAllStatusSignals());

        RobotContainer.orchestra.add(hood, TalonFXOrchestra.Tracks.HOOD);
        RobotContainer.orchestra.add(flywheelGroup.getMotor(0), TalonFXOrchestra.Tracks.FLYWHEEL_LEFT);
        RobotContainer.orchestra.add(flywheelGroup.getMotor(1), TalonFXOrchestra.Tracks.FLYWHEEL_RIGHT);
    }

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelGroup.applyConfig(configuration);
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        hood.getConfigurator().apply(configuration);
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        flywheelGroup.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        hood.setPosition(newValueRotations);
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        flywheelGroup.setControl(request);
    }

    @Override
    public void setHoodControl(ControlRequest request) {
        hood.setControl(request);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelGroup.periodic();
        if (flywheelGroup.hasLostPosition()) { // position doesn't matter
            flywheelGroup.setPosition(0);
        }

        inputs.flywheelPositionMeters = flywheelGroup.getAveragePosition();
        inputs.flywheelVelocityMps =
                SimpleMath.average(flywheelGroup.getVelocities()).orElse(0);
        inputs.flywheelVoltage = SimpleMath.average(flywheelGroup.getVoltages()).orElse(0);
        inputs.flywheelCurrentDraw = Arrays.stream(flywheelGroup.getCurrents()).reduce(Amps.zero(), Current::plus);

        inputs.hoodConnected = hoodPositionSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.hoodPositionRotations = hoodPositionSignal.getValueAsDouble();
        inputs.hoodVelocityRotationsPerSecond = hoodVelocitySignal.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltageSignal.getValueAsDouble();
        inputs.hoodCurrentDraw = hoodCurrentSignal.getValue();
        inputs.hoodRotorFault = hoodRotorFault1Signal.getValue() || hoodRotorFault2Signal.getValue();
    }

    @Override
    public void close() {
        flywheelGroup.close();
        hood.close();
    }
}
