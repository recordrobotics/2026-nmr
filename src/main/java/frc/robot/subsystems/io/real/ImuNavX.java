package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.io.ImuIO;

public class ImuNavX implements ImuIO {

    private static final double G = 9.81;

    private final AHRS nav = new AHRS(NavXComType.kUSB1);
    private final boolean inverted;

    public ImuNavX(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void reset() {
        nav.reset();
    }

    @Override
    public void resetDisplacement() {
        nav.resetDisplacement();
    }

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = nav.isConnected();
        inputs.yaw = Rotation2d.fromDegrees(inverted ? -nav.getAngle() : nav.getAngle());
        inputs.pitch = Rotation2d.fromDegrees(nav.getPitch());
        inputs.roll = Rotation2d.fromDegrees(nav.getRoll());
        inputs.yawRate = DegreesPerSecond.of(inverted ? -nav.getRate() : nav.getRate());
        inputs.pitchRate = DegreesPerSecond.of(0);
        inputs.rollRate = DegreesPerSecond.of(0);
        inputs.worldLinearAccelX = MetersPerSecondPerSecond.of(G * nav.getWorldLinearAccelX());
        inputs.worldLinearAccelY = MetersPerSecondPerSecond.of(G * nav.getWorldLinearAccelY());
    }

    @Override
    public void close() {
        nav.close();
    }
}
