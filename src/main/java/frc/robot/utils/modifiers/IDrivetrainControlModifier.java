package frc.robot.utils.modifiers;

public interface IDrivetrainControlModifier {

    enum Priority {
        /**
         * Lowest priority, typically for automatic control (e.g. path following). This priority
         * should yield to all other priorities.
         */
        AUTO(0),
        /**
         * Medium priority, typically for driver-assist features (e.g. anti-tip, auto-balance, game piece align). This
         * priority should yield to override, but take precedence over automatic control.
         */
        ASSIST(10),
        /**
         * Highest priority, typically for direct control. This priority should take
         * precedence over all other priorities.
         */
        OVERRIDE(99);

        private final int value;

        Priority(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    /**
     * Apply the modifier to the given drivetrain control input.
     *
     * @param control the DrivetrainControl object representing the current control input to be modified
     * @return true if the modifier was applied, false otherwise
     */
    boolean apply(DrivetrainControl control);

    boolean isEnabled();

    void setEnabled(boolean enabled);

    /**
     * Debug method that logs state before and after applying the modifier.
     * @param control the DrivetrainControl object representing the current control input to be modified
     * @return true if the modifier was applied, false otherwise
     */
    default boolean logApply(DrivetrainControl control) {
        control.logState("IDrivetrainControlModifier/" + getClass().getSimpleName() + "/Before/");
        boolean applied = apply(control);
        control.logState("IDrivetrainControlModifier/" + getClass().getSimpleName() + "/After/");
        return applied;
    }
}
