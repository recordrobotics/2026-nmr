package frc.robot.tests;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;
import java.util.EnumMap;
import java.util.Map;

public class TestControlBridge implements AbstractControl {

    private static final double HALF_SPEED_DIRECTIONAL_DIVIDER = 3;
    private static final double HALF_SPEED_SPIN_DIVIDER = 2;

    private static TestControlBridge instance;

    static {
        RobotContainer.setTestControl(getInstance());
    }

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    private EnumMap<Button, Integer> buttonStates = new EnumMap<>(Button.class);
    private EnumMap<Axis, Double> axisStates = new EnumMap<>(Axis.class);
    private EnumMap<RumbleType, Double> rumbleStates = new EnumMap<>(RumbleType.class);

    public static TestControlBridge getInstance() {
        if (instance == null) {
            instance = new TestControlBridge();
        }
        return instance;
    }

    private boolean getButton(Button btn) {
        int state = buttonStates.getOrDefault(btn, 0);
        // state > 0 is being pressed, state < 0 is being held down, state == 0 is released
        return state != 0;
    }

    @Override
    public void update() {
        for (Map.Entry<Button, Integer> entry : buttonStates.entrySet()) {
            int state = entry.getValue();

            if (state > 0) {
                entry.setValue(state - 1); // Decrement the button state
            } else if (state < 0) {
                entry.setValue(-1); // Button is held down
            }
        }

        Pair<Double, Double> xy = getXYOriented();

        double x = xy.getFirst() * getDirectionalSpeedLevel();
        double y = xy.getSecond() * getDirectionalSpeedLevel();

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
        acceleration = new Transform2d(
                        velocity.getTranslation()
                                .minus(lastVelocity.getTranslation())
                                .div(0.02),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(0.02);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(0.02),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(0.02);

        lastVelocity = velocity;
        lastAcceleration = acceleration;
    }

    @Override
    public DrivetrainControl getDrivetrainControl() {
        return DrivetrainControl.createFieldRelative(
                velocity,
                acceleration,
                jerk,
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
    }

    @Override
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXYRaw();
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
    }

    public Pair<Double, Double> getXYRaw() {
        double x = SimpleMath.applyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.X, 0.0),
                Constants.Control.JOYSTICK_XY_THRESHOLD,
                Constants.Control.JOYSTICK_XY_SENSITIVITY);
        double y = SimpleMath.applyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.Y, 0.0),
                Constants.Control.JOYSTICK_XY_THRESHOLD,
                Constants.Control.JOYSTICK_XY_SENSITIVITY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(axisStates.getOrDefault(Axis.TWIST, 0.0), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public boolean isHalfSpeedTriggered() {
        return false; // half speed auto enabled when scoring
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
                1,
                -1,
                Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
                Constants.Swerve.MAX_MODULE_SPEED);

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_DIRECTIONAL_DIVIDER;
        }

        return speed;
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
                1,
                -1,
                Constants.Control.SPIN_SPEED_METER_LOW,
                Constants.Swerve.MAX_ANGULAR_SPEED_RADIANS);

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_SPIN_DIVIDER;
        }

        return speed;
    }

    @Override
    public boolean isPoseResetTriggered() {
        return getButton(Button.POSE_RESET);
    }

    @Override
    public boolean isKillTriggered() {
        return getButton(Button.KILL);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        rumbleStates.put(type, value);
    }

    public double getRumble(RumbleType type) {
        return rumbleStates.getOrDefault(type, 0.0);
    }

    /**
     * Holds a button indefinitely until released {@link #releaseButton(Button)}.
     * @param btn the button to hold
     */
    public void holdButton(Button btn) {
        buttonStates.put(btn, -1);
    }

    /**
     * Releases a button.
     * @param btn the button to release
     */
    public void releaseButton(Button btn) {
        buttonStates.put(btn, 0);
    }

    /**
     * Presses a button for a certain number of cycles.
     * The button will be pressed for the specified number of cycles, after which it will be released.
     * @param btn the button to press
     * @param cycles the number of cycles to press the button for
     */
    public void pressButton(Button btn, int cycles) {
        // add 1 to cycle count because of update order - control periodic (where cycles is decremented) is called
        // before trigger periodic where value is read
        buttonStates.put(btn, cycles + 1);
    }

    /**
     * Sets the value of an axis.
     * @param axis the axis to set
     * @param value the value to set the axis to
     */
    public void setAxis(Axis axis, double value) {
        axisStates.put(axis, value);
    }

    /**
     * Resets the control bridge state.
     * This method resets all button states, axis states, and motion states to their initial values.
     */
    public void reset() {
        lastVelocity = new Transform2d();
        lastAcceleration = new Transform2d();
        velocity = new Transform2d();
        acceleration = new Transform2d();
        jerk = new Transform2d();

        buttonStates.clear();
        axisStates.clear();
    }

    public enum Button {
        POSE_RESET,
        LIMELIGHT_RESET,
        KILL
    }

    public enum Axis {
        X,
        Y,
        TWIST,
        SPEED_LEVEL,
        POV
    }

    @Override
    public String toDisplayName() {
        throw new UnsupportedOperationException(
                "TestControlBridge should only be used for automated tests, never as a control scheme.");
    }

    @Override
    public boolean isIntakeInvertPressed() {
        return false; // TODO
    }

    @Override
    public boolean isIntakePressed() {
        return false; // TODO
    }

    @Override
    public boolean isIntakeUpPressed() {
        return false; // TODO
    }

    @Override
    public boolean isClimbPressed() {
        return false; // TODO
    }

    @Override
    public boolean isShooterPassPressed() {
        return false; // TODO
    }

    @Override
    public boolean isShooterDisableShootPressed() {
        return false; // TODO
    }

    @Override
    public boolean isUnstuckSpindexerPressed() {
        return false; // TODO
    }

    @Override
    public boolean isReverseIntakePressed() {
        return false; // TODO
    }

    @Override
    public boolean isSlowSpeedPressed() {
        return false; // TODO
    }

    @Override
    public boolean isDefenseModePressed() {
        return false; // TODO
    }

    @Override
    public boolean isIntakeRelativePressed() {
        return false; // TODO
    }

    @Override
    public boolean hasUserInput() {
        // Check if any button is pressed or any axis is non-zero
        for (Integer state : buttonStates.values()) {
            if (state != 0) {
                return true;
            }
        }

        for (Double value : axisStates.values()) {
            if (value != 0.0) {
                return true;
            }
        }

        return false;
    }
}
