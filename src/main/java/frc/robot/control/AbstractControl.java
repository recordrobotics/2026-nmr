package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.utils.ContainerUtils;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.modifiers.DrivetrainControl;

public interface AbstractControl {

    void update();

    boolean isIntakeUpPressed();

    boolean isIntakePressed();

    boolean isIntakeInvertPressed();

    boolean isReverseIntakePressed();

    boolean isDefenseModePressed();

    boolean isClimbPressed();

    boolean isShooterPassPressed();

    boolean isShooterDisableShootPressed();

    boolean isUnstuckSpindexerPressed();

    boolean isIntakeRelativePressed();

    // Movement
    DrivetrainControl getDrivetrainControl();

    Transform2d getRawDriverInput();

    // Misc
    boolean isPoseResetTriggered();

    boolean isKillTriggered();

    boolean isSlowSpeedPressed();

    void vibrate(RumbleType type, double value);

    String toDisplayName();

    /**
     * Checks if there is any active user input
     * @return true if there is user input on any axis or button, false otherwise
     */
    boolean hasUserInput();

    // Orient XY
    static Pair<Double, Double> orientXY(Pair<Double, Double> input) {
        double inputX = input.getFirst();
        double inputY = input.getSecond();

        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return new Pair<>(-inputY, -inputX);
        else return new Pair<>(inputY, inputX);
    }

    /**
     * Checks if the given HID has any active user input (axes or buttons) that is not in the ignoreAxis list.
     * @param hid the HID to check for user input
     * @param ignoreAxis a varargs list of axis indices to ignore when checking for user input
     * @return true if there is user input on any axis (not in ignoreAxis) or button, false otherwise
     */
    static boolean hasUserInput(GenericHID hid, int... ignoreAxis) {
        // Check axes
        for (int i = 0; i < hid.getAxisCount(); i++) {
            if (Math.abs(hid.getRawAxis(i)) > 0.3 && !ContainerUtils.contains(ignoreAxis, i)) {
                return true;
            }
        }

        // Check buttons
        for (int i = 1; i <= hid.getButtonCount(); i++) {
            if (hid.getRawButton(i)) {
                return true;
            }
        }

        return false;
    }
}
