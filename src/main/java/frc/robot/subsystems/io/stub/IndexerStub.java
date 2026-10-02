package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.IndexerIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class IndexerStub implements IndexerIO {

    public IndexerStub(double periodicDt) {}

    public IndexerStub() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
