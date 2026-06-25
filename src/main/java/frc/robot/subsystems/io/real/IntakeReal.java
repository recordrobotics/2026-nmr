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
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import frc.robot.utils.TalonFXOrchestra;
import java.util.Arrays;

public class IntakeReal implements IntakeIO {

    protected final TalonFX wheel;
    protected final TalonFXMotorGroup armGroup;

    private final StatusSignal<Angle> wheelPositionSignal;
    private final StatusSignal<AngularVelocity> wheelVelocitySignal;
    private final StatusSignal<Current> wheelCurrentSignal;
    private final StatusSignal<Voltage> wheelVoltageSignal;

    public IntakeReal() {
        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);
        wheel.optimizeBusUtilization();
        armGroup = new TalonFXMotorGroup(
                "Intake Arm",
                true,
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));

        wheelPositionSignal = wheel.getPosition();
        wheelVelocitySignal = wheel.getVelocity();
        wheelCurrentSignal = wheel.getSupplyCurrent();
        wheelVoltageSignal = wheel.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), wheelVelocitySignal);
        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), armGroup.getAllHighRefreshRateStatusSignals());

        RobotContainer.allStatusSignalsToRefresh.addAll(
                wheelPositionSignal, wheelVelocitySignal, wheelVoltageSignal, wheelCurrentSignal);
        RobotContainer.allStatusSignalsToRefresh.addAll(armGroup.getAllStatusSignals());

        RobotContainer.orchestra.add(wheel, TalonFXOrchestra.Tracks.INTAKE_WHEEL);
        RobotContainer.orchestra.add(armGroup.getMotor(0), TalonFXOrchestra.Tracks.INTAKE_ARM_LEFT);
        RobotContainer.orchestra.add(armGroup.getMotor(1), TalonFXOrchestra.Tracks.INTAKE_ARM_RIGHT);
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        armGroup.applyConfig(configuration);
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheel.getConfigurator().apply(configuration);
    }

    @Override
    public void setArmControl(ControlRequest request) {
        armGroup.setControl(request);
    }

    @Override
    public void setWheelControl(ControlRequest request) {
        wheel.setControl(request);
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        armGroup.setPosition(newValue);
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        wheel.setPosition(newValue);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        armGroup.periodic();

        inputs.armHasPosition = !armGroup.hasLostPosition();
        inputs.armPositionRotations = armGroup.getAveragePosition();
        inputs.armVelocityRotationsPerSecond =
                SimpleMath.average(armGroup.getVelocities()).orElse(0);
        inputs.armVoltage = SimpleMath.average(armGroup.getVoltages()).orElse(0);
        inputs.armCurrentDraw = Arrays.stream(armGroup.getCurrents()).reduce(Amps.zero(), Current::plus);

        inputs.wheelConnected = wheelVelocitySignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.wheelPositionMeters = wheelPositionSignal.getValueAsDouble();
        inputs.wheelVelocityMps = wheelVelocitySignal.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltageSignal.getValueAsDouble();
        inputs.wheelCurrentDraw = wheelCurrentSignal.getValue();
    }

    @Override
    public void close() {
        wheel.close();
        armGroup.close();
    }
}
