package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public final class SysIdManager {

    private SysIdManager() {}

    /**
     * Returns the current SysIdProvider instance.
     * <hr/>
     * <ul>
     * <li>Example:  {@link SysIdProvider#NONE} is the default provider that does nothing.</li>
     * <li>Example: {@code new CoralIntake.SysIdArm()} is a provider that runs SysId on the CoralIntake.SysIdArm.</li>
     * </ul>
     * @return the current SysIdProvider instance.
     */
    public static SysIdProvider getProvider() {
        return SysIdProvider.NONE;
    }

    public interface SysIdProvider {
        double IN_BETWEEN_STEP_DELAY = 0.4;

        SysIdProvider NONE = new SysIdProvider() {
            @Override
            public Command sysIdQuasistatic(Direction direction) {
                return Commands.none();
            }

            @Override
            public Command sysIdDynamic(Direction direction) {
                return Commands.none();
            }

            @Override
            public boolean isEnabled() {
                return false;
            }

            @Override
            public boolean isReversed() {
                return false;
            }
        };

        Command sysIdQuasistatic(Direction direction);

        Command sysIdDynamic(Direction direction);

        boolean isEnabled();

        boolean isReversed();

        default Command createCommand() {
            return new InstantCommand()
                    .andThen(sysIdQuasistatic(isReversed() ? Direction.kReverse : Direction.kForward)
                            .andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                    .andThen(sysIdQuasistatic(isReversed() ? Direction.kForward : Direction.kReverse)
                            .andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                    .andThen(sysIdDynamic(isReversed() ? Direction.kReverse : Direction.kForward)
                            .andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                    .andThen(sysIdDynamic(isReversed() ? Direction.kForward : Direction.kReverse)
                            .andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)));
        }
    }
}
