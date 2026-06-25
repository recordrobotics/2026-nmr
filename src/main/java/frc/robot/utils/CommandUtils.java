package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public final class CommandUtils {
    private CommandUtils() {}

    /**
     * Wraps a command to ensure that it finishes when interrupted. This is useful for commands that
     * may not finish on their own when interrupted, such as commands that wait for a condition or
     * timeout
     *
     * @param command the command to wrap
     * @return a new WrapperCommand that finishes when interrupted
     */
    public static WrapperCommand finishOnInterrupt(Command command) {
        return new WrapperCommand(command) {
            private boolean finished = false;

            @Override
            public void initialize() {
                finished = false;
                super.initialize();
            }

            @Override
            public boolean isFinished() {
                return super.isFinished() || finished;
            }

            @Override
            public void end(boolean interrupted) {
                finished = true;
                super.end(interrupted);
            }
        };
    }

    @SuppressWarnings("java:S2301") // this is a conditional method
    public static Command maybeProxy(boolean doProxy, Command command) {
        return doProxy ? command.asProxy() : command;
    }

    public static Command setForceDisabledForAllCommand(
            boolean forceDisabled, int cyclesBeforeEnable, KillableSubsystem... subsystems) {
        return Commands.runOnce(
                () -> KillableSubsystem.setForceDisabledForAll(forceDisabled, cyclesBeforeEnable, subsystems),
                subsystems);
    }
}
