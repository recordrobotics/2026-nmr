package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.IntakeIO;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class IntakeStub implements IntakeIO {

    public IntakeStub(double periodicDt, SwerveDriveSimulation swerveDriveSimulation) {}

    public IntakeStub() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setArmControl(ControlRequest request) {}

    @Override
    public void setWheelControl(ControlRequest request) {}

    @Override
    public void setWheelPositionMeters(double newValue) {}

    @Override
    public void setArmPositionRotations(double newValue) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
