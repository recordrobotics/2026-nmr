package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.subsystems.io.ImuIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ImuStub implements ImuIO {

    public ImuStub(double periodicDt) {}

    public ImuStub() {}

    @Override
    public void updateInputs(ImuIOInputs inputs) {}

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {}

    @Override
    public void reset() {}

    @Override
    public void resetDisplacement() {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
