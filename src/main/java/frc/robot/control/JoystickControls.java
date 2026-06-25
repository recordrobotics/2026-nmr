package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

@SuppressWarnings({"java:S109"})
public class JoystickControls implements AbstractControl {

    private Joystick joystick;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    public JoystickControls(int joystickPort) {
        joystick = new Joystick(joystickPort);
    }

    @Override
    public void update() {
        Pair<Double, Double> xy = getXYOriented();

        double x = xy.getFirst() * getDirectionalSpeedLevel();
        double y = xy.getSecond() * getDirectionalSpeedLevel();

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
        acceleration = new Transform2d(
                        velocity.getTranslation()
                                .minus(lastVelocity.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);

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

    @Override
    public boolean isReverseIntakePressed() {
        return false;
    }

    @Override
    public boolean isSlowSpeedPressed() {
        return false;
    }

    public Pair<Double, Double> getXYRaw() {
        double unsquaredX = SimpleMath.applyThresholdAndSensitivity(
                joystick.getX(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);
        double unsquaredY = SimpleMath.applyThresholdAndSensitivity(
                joystick.getY(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);

        // Squares the inputs while preserving the sign to allow for finer control at low speeds
        double x = Math.copySign(Math.pow(unsquaredX, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredX);
        double y = Math.copySign(Math.pow(unsquaredY, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public Double getSpin() {
        // Gets raw twist value
        double unsquaredSpin = SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
        // Squares the input while preserving the sign to allow for finer control at low speeds
        return Math.copySign(Math.pow(unsquaredSpin, Constants.Control.JOYSTICK_SPIN_EXPONENT), unsquaredSpin);
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
                Constants.Swerve.MAX_MODULE_SPEED);

        return speed;
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> MAX, then returns
        double speed = SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.SPIN_SPEED_METER_LOW,
                Constants.Swerve.MAX_ANGULAR_SPEED_RADIANS / 3);

        return speed;
    }

    @Override
    public boolean isPoseResetTriggered() {
        return joystick.getRawButtonPressed(5);
    }

    @Override
    public boolean isKillTriggered() {
        return joystick.getRawButton(4);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        // Joystick does not support rumble, so this method is empty
    }

    @Override
    public boolean isIntakeInvertPressed() {
        return joystick.getRawButton(2);
    }

    @Override
    public boolean isIntakePressed() {
        return false;
    }

    @Override
    public boolean isIntakeUpPressed() {
        return false;
    }

    @Override
    public boolean isClimbPressed() {
        return joystick.getRawButton(7);
    }

    @Override
    public boolean isShooterPassPressed() {
        return joystick.getRawButton(1);
    }

    @Override
    public boolean isShooterDisableShootPressed() {
        return joystick.getRawButton(10);
    }

    @Override
    public boolean isUnstuckSpindexerPressed() {
        return joystick.getRawButton(3);
    }

    @Override
    public boolean isDefenseModePressed() {
        return joystick.getRawButton(11);
    }

    @Override
    public boolean isIntakeRelativePressed() {
        return false;
    }

    @Override
    public boolean hasUserInput() {
        // ignores speed level
        return AbstractControl.hasUserInput(joystick, 3);
    }

    @Override
    public String toDisplayName() {
        return "Joystick";
    }
}
