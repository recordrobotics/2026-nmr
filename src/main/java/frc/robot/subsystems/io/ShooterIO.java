package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class ShooterIOInputs {
        public double flywheelPositionMeters = 0;
        public double flywheelVelocityMps = 0;
        public double flywheelVoltage = 0;
        public Current flywheelCurrentDraw = Amps.zero();

        public boolean hoodConnected = false;
        public double hoodPositionRotations = 0;
        public double hoodVelocityRotationsPerSecond = 0;
        public double hoodVoltage = 0;
        public Current hoodCurrentDraw = Amps.zero();
        public boolean hoodRotorFault = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {}

    default void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {}

    default void setFlywheelControl(ControlRequest request) {}

    default void setHoodControl(ControlRequest request) {}

    default void setHoodPositionRotations(double newValue) {}

    default void setFlywheelPositionMeters(double newValue) {}

    default void clearRotorFault() {}

    default void close() {}

    default void simulationPeriodic() {}
}
