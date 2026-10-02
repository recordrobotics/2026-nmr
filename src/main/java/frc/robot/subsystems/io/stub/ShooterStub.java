package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.ShooterIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ShooterStub implements ShooterIO {

    public ShooterStub(double periodicDt) {}

    public ShooterStub() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setFlywheelControl(ControlRequest request) {}

    @Override
    public void setFlywheelPositionMeters(double newValue) {}

    @Override
    public void clearRotorFault() {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
