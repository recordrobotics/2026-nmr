package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@SuppressWarnings("java:S6548") // We want this to be a singleton
public class SubsystemManager extends SubsystemBase {

    private static final boolean USE_MULTITHREADING = false;

    /** The Singleton Instance. */
    private static SubsystemManager instance;

    // A map from subsystems registered with the scheduler to their default commands.  Also used
    // as a list of currently-registered subsystems.
    @SuppressWarnings("java:S6411") // Using ManagedSubsystemBase as Key is inefficient but acceptable
    private final Map<ManagedSubsystemBase, Command> subsystems = new LinkedHashMap<>();

    private final Watchdog watchdog = new Watchdog(TimedRobot.kDefaultPeriod, () -> {});

    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final List<Future<?>> futures = new ArrayList<>();

    /**
     * Returns the Scheduler instance.
     *
     * @return the instance
     */
    public static synchronized SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    /**
     * Registers subsystems with the scheduler. This must be called for the subsystem's periodic block
     * to run when the scheduler is run, and for the subsystem's default command to be scheduled. It
     * is recommended to call this from the constructor of your subsystem implementations.
     *
     * @param subsystems the subsystem to register
     */
    public void registerSubsystem(ManagedSubsystemBase... subsystems) {
        for (ManagedSubsystemBase subsystem : subsystems) {
            if (subsystem == null) {
                ConsoleLogger.logWarning("Tried to register a null subsystem");
            } else if (this.subsystems.containsKey(subsystem)) {
                ConsoleLogger.logWarning("Tried to register an already-registered subsystem");
            } else {
                this.subsystems.put(subsystem, null);
            }
        }
    }

    @Override
    public void periodic() {
        watchdog.reset();

        if (USE_MULTITHREADING) {
            futures.clear();
        }

        // Run the periodic method of all registered subsystems.
        for (ManagedSubsystemBase subsystem : subsystems.keySet()) {
            if (USE_MULTITHREADING) {
                futures.add(executor.submit(subsystem::periodicManaged));
            } else {
                subsystem.periodicManaged();
            }
            if (RobotBase.isSimulation()) {
                if (USE_MULTITHREADING) {
                    futures.add(executor.submit(subsystem::simulationPeriodicManaged));
                } else {
                    subsystem.simulationPeriodicManaged();
                }
            }
        }

        if (USE_MULTITHREADING) {
            for (Future<?> future : futures) {
                try {
                    future.get();
                } catch (ExecutionException | CancellationException e) {
                    ConsoleLogger.logError(
                            "SubsystemManager periodic error", Objects.requireNonNullElse(e.getCause(), e));
                } catch (InterruptedException e) {
                    ConsoleLogger.logError(
                            "SubsystemManager periodic interrupted", Objects.requireNonNullElse(e.getCause(), e));
                    Thread.currentThread().interrupt();
                }
            }
        }

        watchdog.disable();
        if (watchdog.isExpired()) {
            ConsoleLogger.logInfo("SubsystemManager loop overrun: " + watchdog.getTime());
        }
    }
}
