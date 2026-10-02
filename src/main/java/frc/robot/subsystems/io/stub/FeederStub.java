package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.FeederIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class FeederStub implements FeederIO {

    public FeederStub(double periodicDt) {}

    public FeederStub() {}

    @Override
    public void updateInputs(FeederIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
