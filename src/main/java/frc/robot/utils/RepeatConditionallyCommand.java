// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

/**
 * A command that runs another command repeatedly, restarting it when it ends, until this command is
 * interrupted. Command instances that are passed to it cannot be added to any other groups, or
 * scheduled individually.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually,and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public final class RepeatConditionallyCommand extends Command {
    private final Command command;
    private boolean ended;
    private BooleanSupplier condition;
    private boolean runAtLeastOnce;

    /**
     * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
     * ends, until this command is interrupted.
     *
     * @param command the command to run repeatedly
     */
    @SuppressWarnings("this-escape")
    public RepeatConditionallyCommand(Command command, BooleanSupplier condition, boolean runAtLeastOnce) {
        this.command = requireNonNullParam(command, "command", "RepeatConditionallyCommand");
        this.condition = condition;
        this.runAtLeastOnce = runAtLeastOnce;
        CommandScheduler.getInstance().registerComposedCommands(command);
        addRequirements(command.getRequirements());
        setName("RepeatConditionally(" + command.getName() + ")");
    }

    @Override
    public void initialize() {
        ended = false;
        if (condition.getAsBoolean() || runAtLeastOnce) {
            try {
                command.initialize();
            } catch (Exception e) {
                ConsoleLogger.logError("command.initialize()", e);
            }
        } else {
            ended = true;
        }
    }

    @Override
    public void execute() {
        if (ended && condition.getAsBoolean()) {
            ended = false;
            try {
                command.initialize();
            } catch (Exception e) {
                ConsoleLogger.logError("command.initialize()", e);
                return;
            }
        }

        if (!ended) {
            try {
                command.execute();
            } catch (Exception e) {
                ConsoleLogger.logError("command.execute()", e);
                return;
            }
            try {
                if (command.isFinished()) {
                    // restart command
                    command.end(false);
                    ended = true;
                }
            } catch (Exception e) {
                ConsoleLogger.logError("command.isFinished/end()", e);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command finished in the
        // last call to our execute())
        if (!ended) {
            try {
                command.end(interrupted);
            } catch (Exception e) {
                ConsoleLogger.logError("command.end()", e);
            }
            ended = true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("command", command::getName, null);
    }
}
