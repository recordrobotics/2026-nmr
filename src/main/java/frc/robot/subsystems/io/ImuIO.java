package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class ImuIOInputs {
        public boolean connected = false;

        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;
        public Rotation2d roll = Rotation2d.kZero;

        public AngularVelocity yawRate = DegreesPerSecond.zero();
        public AngularVelocity pitchRate = DegreesPerSecond.zero();
        public AngularVelocity rollRate = DegreesPerSecond.zero();

        public LinearAcceleration worldLinearAccelX = MetersPerSecondPerSecond.zero();
        public LinearAcceleration worldLinearAccelY = MetersPerSecondPerSecond.zero();
    }

    default void updateInputs(ImuIOInputs inputs) {}

    default void applyPigeon2Config(Pigeon2Configuration config) {}

    default void reset() {}

    default void resetDisplacement() {}

    default void close() {}

    default void simulationPeriodic() {}
}
