package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public interface GamepadController {

    double getLeftX();

    double getLeftY();

    double getRightX();

    double getRightY();

    boolean rightStickButtonPressed();

    double leftTriggerAxis();

    double rightTriggerAxis();

    boolean leftBumperPressed();

    boolean rightBumperPressed();

    boolean inputDiamondUp();

    boolean inputDiamondLeft();

    boolean inputDiamondRight();

    boolean inputDiamondDown();

    boolean leftMenuButtonPressed();

    boolean rightMenuButtonPressed();

    int getPOV();

    void vibrate(RumbleType type, double value);

    GenericHID getGenericHID();

    boolean hasUserInput();

    String toDisplayName();
}
