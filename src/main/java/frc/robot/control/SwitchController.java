package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;

public class SwitchController implements GamepadController {

    private final GenericHID gamepad;

    public SwitchController(int switchPort) {
        gamepad = new GenericHID(switchPort);
    }

    @Override
    public double getLeftX() {
        return gamepad.getRawAxis(0);
    }

    @Override
    public double getLeftY() {
        return gamepad.getRawAxis(1);
    }

    @Override
    public double getRightX() {
        return gamepad.getRawAxis(2);
    }

    @Override
    public double getRightY() {
        return gamepad.getRawAxis(3);
    }

    @Override
    public boolean rightStickButtonPressed() {
        return gamepad.getRawButton(12);
    }

    @Override
    public double leftTriggerAxis() {
        return gamepad.getRawButton(7) ? 1.0 : 0.0;
    }

    @Override
    public double rightTriggerAxis() {
        return gamepad.getRawButton(8) ? 1.0 : 0.0;
    }

    @Override
    public boolean leftBumperPressed() {
        return gamepad.getRawButton(5);
    }

    @Override
    public boolean rightBumperPressed() {
        return gamepad.getRawButton(6);
    }

    @Override
    public boolean inputDiamondUp() {
        return gamepad.getRawButton(4);
    }

    @Override
    public boolean inputDiamondLeft() {
        return gamepad.getRawButton(3);
    }

    @Override
    public boolean inputDiamondRight() {
        return gamepad.getRawButton(2);
    }

    @Override
    public boolean inputDiamondDown() {
        return gamepad.getRawButton(1);
    }

    @Override
    public boolean leftMenuButtonPressed() {
        return gamepad.getRawButtonPressed(14);
    }

    @Override
    public boolean rightMenuButtonPressed() {
        return gamepad.getRawButton(13);
    }

    @Override
    public int getPOV() {
        return gamepad.getPOV();
    }

    @Override
    public void vibrate(GenericHID.RumbleType type, double value) {
        gamepad.setRumble(type, value);
    }

    @Override
    public GenericHID getGenericHID() {
        return gamepad;
    }

    @Override
    public String toDisplayName() {
        return "Switch";
    }

    @Override
    public boolean hasUserInput() {
        return AbstractControl.hasUserInput(getGenericHID());
    }
}
