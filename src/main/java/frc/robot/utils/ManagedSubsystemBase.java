package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ManagedSubsystemBase extends SubsystemBase implements AutoCloseable {

    protected ManagedSubsystemBase() {
        SubsystemManager.getInstance().registerSubsystem(this);
    }

    /**
     * This method is called periodically by the {@link ManagedSubsystemBase}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    public void periodicManaged() {}

    /**
     * This method is called periodically by the {@link ManagedSubsystemBase}. Useful for updating
     * subsystem-specific state that needs to be maintained for simulations, such as for updating
     * {@link edu.wpi.first.wpilibj.simulation} classes and setting simulated sensor readings.
     */
    public void simulationPeriodicManaged() {}

    /**
     * NOTE: THIS METHOD IS UNMANAGED AND NOT MULTI-THREADED!<p>
     * ONLY USE THIS IF YOU KNOW WHAT YOU'RE DOING
     * <p>
     * This method is called periodically by the {@link SubsystemBase}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    public void periodicUnmanaged() {}

    /**
     * NOTE: THIS METHOD IS UNMANAGED AND NOT MULTI-THREADED!<p>
     * ONLY USE THIS IF YOU KNOW WHAT YOU'RE DOING
     * <p>
     * This method is called periodically by the {@link SubsystemBase}. Useful for updating
     * subsystem-specific state that needs to be maintained for simulations, such as for updating
     * {@link edu.wpi.first.wpilibj.simulation} classes and setting simulated sensor readings.
     */
    public void simulationPeriodicUnmanaged() {}

    @Override
    public final void periodic() {
        /* prevents inherited usage - managed versions should be used instead */
        periodicUnmanaged();
    }

    @Override
    public final void simulationPeriodic() {
        /* prevents inherited usage - managed versions should be used instead */
        simulationPeriodicUnmanaged();
    }

    @Override
    public void close() {
        // Default implementation does nothing
    }
}
