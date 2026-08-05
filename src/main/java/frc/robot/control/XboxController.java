package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;

public class XboxController implements GamepadController {

    private final edu.wpi.first.wpilibj.XboxController gamepad;

    public XboxController(int xboxPort) {
        gamepad = new edu.wpi.first.wpilibj.XboxController(xboxPort);
    }

    @Override
    public double getLeftX() {
        return gamepad.getLeftX();
    }

    @Override
    public double getLeftY() {
        return gamepad.getLeftY();
    }

    @Override
    public double getRightX() {
        return gamepad.getRightX();
    }

    @Override
    public double getRightY() {
        return gamepad.getRightY();
    }

    @Override
    public boolean rightStickButtonPressed() {
        return gamepad.getRightStickButton();
    }

    @Override
    public double leftTriggerAxis() {
        return gamepad.getLeftTriggerAxis();
    }

    @Override
    public double rightTriggerAxis() {
        return gamepad.getRightTriggerAxis();
    }

    @Override
    public boolean leftBumperPressed() {
        return gamepad.getLeftBumperButton();
    }

    @Override
    public boolean rightBumperPressed() {
        return gamepad.getRightBumperButton();
    }

    @Override
    public boolean inputDiamondUp() {
        return gamepad.getYButton();
    }

    @Override
    public boolean inputDiamondLeft() {
        return gamepad.getXButton();
    }

    @Override
    public boolean inputDiamondRight() {
        return gamepad.getBButton();
    }

    @Override
    public boolean inputDiamondDown() {
        return gamepad.getAButton();
    }

    @Override
    public boolean leftMenuButtonPressed() {
        return gamepad.getRawButtonPressed(7);
    }

    @Override
    public boolean rightMenuButtonPressed() {
        return gamepad.getRawButton(8);
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
        return "Xbox";
    }

    @Override
    public boolean hasUserInput() {
        return AbstractControl.hasUserInput(getGenericHID());
    }
}
