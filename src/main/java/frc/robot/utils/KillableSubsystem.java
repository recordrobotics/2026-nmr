package frc.robot.utils;

public abstract class KillableSubsystem extends ManagedSubsystemBase {

    /**
     * Whether this subsystem is currently force disabled.
     */
    private boolean forceDisabled = false;

    /**
     * The number of periodic cycles before the subsystem automatically re-enables.
     */
    private int cyclesBeforeEnable = 0;

    /**
     * Sets whether this subsystem should be force disabled. When a subsystem is force disabled, it should ignore all commands and set its outputs to a safe state (e.g. zero voltage).
     * @param forceDisabled whether this subsystem should be force disabled.
     * @param cyclesBedforeEnable the number of periodic cycles before the subsystem automatically re-enables. If zero or negative, the subsystem will remain force disabled until manually re-enabled.
     */
    public void setForceDisabled(boolean forceDisabled, int cyclesBedforeEnable) {
        boolean wasForceDisabled = this.forceDisabled;
        this.forceDisabled = forceDisabled;
        this.cyclesBeforeEnable = cyclesBedforeEnable;

        if (wasForceDisabled != forceDisabled) {
            onForceDisabledChange(forceDisabled);
        }
    }

    /**
     * Returns whether this subsystem is currently force disabled. When a subsystem is force disabled, it should ignore all commands and set its outputs to a safe state (e.g. zero voltage).
     * @return whether this subsystem is currently force disabled.
     */
    public boolean isForceDisabled() {
        return forceDisabled;
    }

    /**
     * Returns the number of periodic cycles before the subsystem automatically re-enables. If zero or negative, the subsystem will remain force disabled until manually re-enabled.
     * @return the number of periodic cycles before the subsystem automatically re-enables.
     */
    public int getCyclesBeforeEnable() {
        return cyclesBeforeEnable;
    }

    @Override
    public void periodicUnmanaged() {
        if (forceDisabled && cyclesBeforeEnable > 0) {
            cyclesBeforeEnable--;
            if (cyclesBeforeEnable <= 0) {
                setForceDisabled(false, 0);
            }
        }
    }

    /**
     * Called when the force disabled state of this subsystem changes. Subclasses can override this method to perform actions when the force disabled state changes (e.g. setting motor outputs to zero when force disabled).
     * @param isNowForceDisabled the new force disabled state of this subsystem.
     */
    protected void onForceDisabledChange(boolean isNowForceDisabled) {}

    /**
     * Sets the force disabled state for multiple subsystems at once.
     * @param forceDisabled the new force disabled state to set for all subsystems.
     * @param cyclesBeforeEnable the number of periodic cycles before the subsystems automatically re-enable.
     * @param subsystems the subsystems to set the force disabled state for.
     */
    public static void setForceDisabledForAll(
            boolean forceDisabled, int cyclesBeforeEnable, KillableSubsystem... subsystems) {
        for (KillableSubsystem subsystem : subsystems) {
            subsystem.setForceDisabled(forceDisabled, cyclesBeforeEnable);
        }
    }
}
