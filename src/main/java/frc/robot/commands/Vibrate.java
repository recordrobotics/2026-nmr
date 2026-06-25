package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Vibrate extends Command {

    private RumbleType type;
    private double value;

    public Vibrate(RumbleType type, double value) {
        this.type = type;
        this.value = value;
    }

    @Override
    public void initialize() {
        RobotContainer.getControl().vibrate(type, value);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getControl().vibrate(RumbleType.kBothRumble, 0);
    }
}
