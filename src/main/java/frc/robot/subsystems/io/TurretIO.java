package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class TurretIOInputs {
        public boolean connected = false;
        public double positionRotations = 0;
        public double velocityRotationsPerSecond = 0;
        public double voltage = 0;
        public Current currentDraw = Amps.zero();

        public boolean forwardSoftLimitHit = false;
        public boolean reverseSoftLimitHit = false;

        public LimitSwitchStates limitSwitchStates = LimitSwitchStates.NO_HITS;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    default void setControl(ControlRequest request) {}

    default void setPositionRotations(double newValue) {}

    default void close() {}

    default void simulationPeriodic() {}

    record LimitSwitchStates(boolean frontLeft, boolean backLeft, boolean backRight) {

        public static final LimitSwitchStates NO_HITS = new LimitSwitchStates(false, false, false);

        public boolean isAnyHit() {
            return frontLeft || backLeft || backRight;
        }

        public boolean hasFault() {
            // If more than one switch is hit at the same time, it's likely a fault (e.g. wiring issue)
            return (frontLeft ? 1 : 0) + (backLeft ? 1 : 0) + (backRight ? 1 : 0) > 1;
        }
    }
}
