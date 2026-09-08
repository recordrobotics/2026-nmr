package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import frc.robot.utils.TalonFXOrchestra;
import java.util.Arrays;

public class ShooterReal implements ShooterIO {

    protected final TalonFXMotorGroup flywheelGroup;

    public ShooterReal() {
        flywheelGroup = new TalonFXMotorGroup(
                "Shooter",
                false,
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));

        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), flywheelGroup.getAllHighRefreshRateStatusSignals());

        RobotContainer.allStatusSignalsToRefresh.addAll(flywheelGroup.getAllStatusSignals());

        RobotContainer.orchestra.add(flywheelGroup.getMotor(0), TalonFXOrchestra.Tracks.FLYWHEEL_LEFT);
        RobotContainer.orchestra.add(flywheelGroup.getMotor(1), TalonFXOrchestra.Tracks.FLYWHEEL_RIGHT);
    }

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelGroup.applyConfig(configuration);
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        flywheelGroup.setPosition(newValue);
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        flywheelGroup.setControl(request);
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
    }

    @Override
    public void close() {
        flywheelGroup.close();
    }
}
