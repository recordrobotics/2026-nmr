package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class IntakeIOInputs {
        public boolean armHasPosition = false;
        public double armPositionRotations = 0;
        public double armVelocityRotationsPerSecond = 0;
        public double armVoltage = 0;
        public Current armCurrentDraw = Amps.zero();

        public boolean wheelConnected = false;
        public double wheelPositionMeters = 0;
        public double wheelVelocityMps = 0;
        public double wheelVoltage = 0;
        public Current wheelCurrentDraw = Amps.zero();
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void applyArmTalonFXConfig(TalonFXConfiguration configuration) {}

    default void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {}

    default void setArmControl(ControlRequest request) {}

    default void setWheelControl(ControlRequest request) {}

    default void setWheelPositionMeters(double newValue) {}

    default void setArmPositionRotations(double newValue) {}

    default void close() {}

    default void simulationPeriodic() {}
}
