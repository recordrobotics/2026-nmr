package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Milliamps;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.real.ImuPigeon2;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class ImuSimPigeon2 extends ImuPigeon2 {

    private final GyroSimulation gyroSimulation;
    private final Pigeon2SimState pigeonSimState;

    public ImuSimPigeon2(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;

        this.pigeonSimState = pigeon.getSimState();

        RobotContainer.pdp.registerSimMiniPdpDevice(() -> Milliamps.of(40));
    }

    @Override
    public void reset() {
        gyroSimulation.setRotation(Rotation2d.kZero);
        pigeonSimState.setRawYaw(gyroSimulation.getGyroReading().getMeasure());
        super.reset();
    }

    @Override
    public void simulationPeriodic() {
        Pose3d robotPose = RobotContainer.model.getRobot();

        pigeonSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        pigeonSimState.setRawYaw(gyroSimulation.getGyroReading().getMeasure());
        pigeonSimState.setPitch(robotPose.getRotation().getMeasureY());
        pigeonSimState.setRoll(robotPose.getRotation().getMeasureX());
        pigeonSimState.setAngularVelocityX(0);
        pigeonSimState.setAngularVelocityY(0);
        pigeonSimState.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity());
    }
}
