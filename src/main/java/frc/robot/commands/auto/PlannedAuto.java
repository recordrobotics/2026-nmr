package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ConsoleLogger;

public final class PlannedAuto {
    private static AutoSupplier autoSupplier = null;

    private PlannedAuto() {}

    public static void setAutoSupplier(AutoSupplier supplier) {
        autoSupplier = supplier;
    }

    public static Command getAutoCommand() {
        if (autoSupplier == null) {
            ConsoleLogger.logError("No auto supplier set! Defaulting to do nothing auto.");
            return Commands.none();
        }

        return autoSupplier.getAutoCommand();
    }

    @FunctionalInterface
    public interface AutoSupplier {
        Command getAutoCommand();
    }
}
