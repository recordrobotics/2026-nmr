package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ImuIO;

public class ImuPigeon2 implements ImuIO {

    protected final Pigeon2 pigeon;

    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> angularVelocityXSignal;
    private final StatusSignal<AngularVelocity> angularVelocityYSignal;
    private final StatusSignal<AngularVelocity> angularVelocityZSignal;
    private final StatusSignal<LinearAcceleration> accelerationXSignal;
    private final StatusSignal<LinearAcceleration> accelerationYSignal;

    public ImuPigeon2() {
        this.pigeon = new Pigeon2(RobotMap.PIGEON_2_ID);
        pigeon.optimizeBusUtilization();

        yawSignal = pigeon.getYaw();
        pitchSignal = pigeon.getPitch();
        rollSignal = pigeon.getRoll();
        angularVelocityXSignal = pigeon.getAngularVelocityXWorld();
        angularVelocityYSignal = pigeon.getAngularVelocityYWorld();
        angularVelocityZSignal = pigeon.getAngularVelocityZWorld();
        accelerationXSignal = pigeon.getAccelerationX();
        accelerationYSignal = pigeon.getAccelerationY();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50),
                yawSignal,
                pitchSignal,
                rollSignal,
                angularVelocityXSignal,
                angularVelocityYSignal,
                angularVelocityZSignal);

        RobotContainer.allStatusSignalsToRefresh.addAll(
                yawSignal,
                pitchSignal,
                rollSignal,
                angularVelocityXSignal,
                angularVelocityYSignal,
                angularVelocityZSignal,
                accelerationXSignal,
                accelerationYSignal);
    }

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {
        pigeon.getConfigurator().apply(config);
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = yawSignal
                .getStatus()
                .isOK(); /* check signal status instead of calling isConnected() to reduce bus wait time */
        inputs.yaw = new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocityZSignal));
        inputs.pitch = new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, angularVelocityYSignal));
        inputs.roll = new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, angularVelocityXSignal));
        inputs.yawRate = angularVelocityZSignal.getValue();
        inputs.pitchRate = angularVelocityYSignal.getValue();
        inputs.rollRate = angularVelocityXSignal.getValue();
        inputs.worldLinearAccelX = accelerationXSignal.getValue();
        inputs.worldLinearAccelY = accelerationYSignal.getValue();
    }

    @Override
    public void close() {
        pigeon.close();
    }
}
