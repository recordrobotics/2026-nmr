package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ImuIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class ImuSimNavX implements ImuIO {

    private final GyroSimulation gyroSimulation;

    public ImuSimNavX(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void reset() {
        gyroSimulation.setRotation(Rotation2d.kZero);
    }

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        Pose3d robotPose = RobotContainer.model.getRobot();

        inputs.connected = true;
        inputs.yaw = gyroSimulation.getGyroReading();
        inputs.pitch = new Rotation2d(robotPose.getRotation().getMeasureY());
        inputs.roll = new Rotation2d(robotPose.getRotation().getMeasureX());
        inputs.yawRate = gyroSimulation.getMeasuredAngularVelocity();
        inputs.pitchRate = DegreesPerSecond.of(0);
        inputs.rollRate = DegreesPerSecond.of(0);
        inputs.worldLinearAccelX = MetersPerSecondPerSecond.of(0);
        inputs.worldLinearAccelY = MetersPerSecondPerSecond.of(0);
    }
}
