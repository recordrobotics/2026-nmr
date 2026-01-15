package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class TalonFXMotorGroup implements AutoCloseable {

    public record MotorConfig(int deviceId, String name, InvertedValue inverted) {}

    private static final class MotorData {
        private final MotorConfig config;
        private final TalonFX device;
        private final Follower followerRequest;

        private final StatusSignal<Angle> positionSignal;
        private final StatusSignal<AngularVelocity> velocitySignal;
        private final StatusSignal<Voltage> voltageSignal;
        private final StatusSignal<Current> currentSignal;
        private final boolean prioritizePosition;

        private boolean lastConnected = false;

        private MotorData(MotorConfig config, TalonFX device, Follower followerRequest, boolean prioritizePosition) {
            this.config = config;
            this.device = device;
            this.followerRequest = followerRequest;
            this.prioritizePosition = prioritizePosition;

            this.device.optimizeBusUtilization();

            positionSignal = this.device.getPosition();
            velocitySignal = this.device.getVelocity();
            voltageSignal = this.device.getMotorVoltage();
            currentSignal = this.device.getSupplyCurrent();
        }

        public boolean isOK() {
            return prioritizePosition
                    ? positionSignal.getStatus().isOK()
                    : velocitySignal.getStatus().isOK();
        }

        public Set<BaseStatusSignal> getStatusSignals() {
            return Set.of(positionSignal, velocitySignal, voltageSignal, currentSignal);
        }

        public Set<BaseStatusSignal> getHighRefreshRateStatusSignals() {
            return Set.of(
                    prioritizePosition ? positionSignal : velocitySignal,
                    voltageSignal /* followers require either dutycycle, voltage, or current signal to follow */);
        }
    }

    private final String groupName;
    private final MotorData[] motors;
    private int leaderIndex = -1;
    private ControlRequest lastControlRequest;
    private boolean lostAllMotors = false;
    private double lastPositionSet = 0;

    private BaseStatusSignal[] allStatusSignalsCache = null;
    private BaseStatusSignal[] allHighRefreshRateStatusSignalsCache = null;

    private final SafeAlert errorAlert = new SafeAlert("", AlertType.kError);
    private final SafeAlert lostAllMotorPositionsAlert = new SafeAlert("", AlertType.kWarning);

    public TalonFXMotorGroup(String groupName, boolean prioritizePosition, MotorConfig... motors) {
        this.groupName = groupName;

        lostAllMotorPositionsAlert.setText(groupName + " lost all motor positions!");

        if (motors.length == 0) {
            throw new IllegalArgumentException("Must have at least one motor in the group");
        }

        this.motors = new MotorData[motors.length];
        for (int i = 0; i < motors.length; i++) {
            this.motors[i] = new MotorData(
                    motors[i],
                    new TalonFX(motors[i].deviceId()),
                    new Follower(-1, MotorAlignmentValue.Aligned),
                    prioritizePosition);
        }

        updateLastConnected();
        leaderIndex = findLeader();
        if (leaderIndex != -1) {
            for (int i = 0; i < this.motors.length; i++) {
                if (i != leaderIndex) {
                    updateFollower(this.motors[i]);
                }
            }
        }
        periodic();
    }

    public void applyConfig(TalonFXConfiguration config) {
        for (MotorData motor : motors) {
            motor.device
                    .getConfigurator()
                    .apply(config.withMotorOutput(config.MotorOutput.withInverted(motor.config.inverted)));
        }
    }

    public void setControl(ControlRequest request) {
        lastControlRequest = request;
        if (leaderIndex != -1) {
            motors[leaderIndex].device.setControl(request);
        }
    }

    public final void updateFollower(MotorData motor) {
        motor.device.setControl(motor.followerRequest
                .withLeaderID(motors[leaderIndex].config.deviceId)
                .withMotorAlignment(
                        motors[leaderIndex].config.inverted == motor.config.inverted
                                ? MotorAlignmentValue.Aligned
                                : MotorAlignmentValue.Opposed));
    }

    public void checkLeaderDisconnect() {
        if (!motors[leaderIndex].isOK()) {
            motors[leaderIndex].lastConnected = false;

            leaderIndex = findLeader();

            if (leaderIndex == -1) {
                lostAllMotors = true;
                return;
            }

            for (int i = 0; i < motors.length; i++) {
                if (i != leaderIndex) {
                    updateFollower(motors[i]);
                }
            }

            setControl(lastControlRequest);
        }
    }

    public void checkFollowerReconnect() {
        for (int i = 0; i < motors.length; i++) {
            if (i != leaderIndex && !motors[i].lastConnected && motors[i].isOK()) {
                motors[i].device.setPosition(getAveragePosition());
                motors[i].lastConnected = true;
                updateFollower(motors[i]);
            }
        }
    }

    public final int findLeader() {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].isOK()) {
                return i;
            }
        }
        return -1;
    }

    public double getAveragePosition() {
        double avg = SimpleMath.average(getPositions()).orElse(lastPositionSet);
        lastPositionSet = avg;
        return avg;
    }

    public final void updateLastConnected() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].lastConnected = motors[i].isOK();
        }
    }

    public double[] getPositions() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.isOK())
                .mapToDouble(m -> m.positionSignal.getValueAsDouble())
                .toArray();
    }

    public double[] getVelocities() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.isOK())
                .mapToDouble(m -> m.velocitySignal.getValueAsDouble())
                .toArray();
    }

    public Current[] getCurrents() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.isOK())
                .map(m -> m.currentSignal.getValue())
                .toArray(Current[]::new);
    }

    public double[] getVoltages() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.isOK())
                .mapToDouble(m -> m.voltageSignal.getValueAsDouble())
                .toArray();
    }

    public void setPosition(double position) {
        if (leaderIndex != -1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].device.setPosition(position);
            }
            lostAllMotors = false;
        }
        lastPositionSet = position;
    }

    public void updateAlert() {
        if (leaderIndex == -1) {
            errorAlert.setText(groupName + " no leader!");
            errorAlert.set(true);
        } else {
            List<String> disconnectedMotors = new ArrayList<>();
            for (int i = 0; i < motors.length; i++) {
                if (!motors[i].isOK()) {
                    disconnectedMotors.add(motors[i].config.name);
                }
            }

            if (!disconnectedMotors.isEmpty()) {
                errorAlert.setText(groupName + " " + String.join(",", disconnectedMotors) + " disconnected!");
                errorAlert.set(true);
            } else {
                errorAlert.set(false);
            }
        }

        lostAllMotorPositionsAlert.set(lostAllMotors);
    }

    public boolean hasLostPosition() {
        return lostAllMotors;
    }

    public final void periodic() {
        if (leaderIndex == -1) {
            leaderIndex = findLeader();
            if (leaderIndex != -1) {
                setControl(lastControlRequest);
            }
        }

        updateAlert();

        if (leaderIndex != -1) {
            checkLeaderDisconnect();
            checkFollowerReconnect();
        } else {
            lostAllMotors = true;
        }

        updateLastConnected();
    }

    public TalonFXSimState getSimState(int index) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new UnsupportedOperationException("Cannot get sim state of real motor");
        }
        return motors[index].device.getSimState();
    }

    public TalonFXSimState[] getSimStates() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new UnsupportedOperationException("Cannot get sim state of real motor");
        }
        return Arrays.stream(motors).map(m -> m.device.getSimState()).toArray(TalonFXSimState[]::new);
    }

    public TalonFX getMotor(int index) {
        return motors[index].device;
    }

    public BaseStatusSignal[] getAllStatusSignals() {
        return getAllStatusSignals(false);
    }

    public BaseStatusSignal[] getAllStatusSignals(boolean refresh) {
        if (refresh || allStatusSignalsCache == null) {
            Set<BaseStatusSignal> set = new HashSet<>();

            for (int i = 0; i < motors.length; i++) {
                set.addAll(motors[i].getStatusSignals());
            }

            allStatusSignalsCache = set.toArray(BaseStatusSignal[]::new);
        }

        return allStatusSignalsCache;
    }

    public BaseStatusSignal[] getAllHighRefreshRateStatusSignals() {
        return getAllHighRefreshRateStatusSignals(false);
    }

    public BaseStatusSignal[] getAllHighRefreshRateStatusSignals(boolean refresh) {
        if (refresh || allHighRefreshRateStatusSignalsCache == null) {
            Set<BaseStatusSignal> set = new HashSet<>();

            for (int i = 0; i < motors.length; i++) {
                set.addAll(motors[i].getHighRefreshRateStatusSignals());
            }

            allHighRefreshRateStatusSignalsCache = set.toArray(BaseStatusSignal[]::new);
        }

        return allHighRefreshRateStatusSignalsCache;
    }

    @Override
    public void close() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].device.close();
        }
    }
}
