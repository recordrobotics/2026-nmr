package utils;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.tests.TestControlBridge;
import frc.robot.utils.ConsoleLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import org.ironmaple.simulation.SimulatedArena;

public class TestRobot {

    static {
        if (!Constants.RobotState.UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE) {
            NetworkTableInstance.getDefault().startLocal(); // disable nt
        }
        Unmanaged.setPhoenixDiagnosticsStartTime(-1); // disable phoenix
        Constants.RobotState.setRunningAsUnitTest();
        RobotController.setTimeSource(TestRobot::getTimestamp);
    }

    private static volatile Robot testRobot;
    private static Thread robotThread;
    private static final ReentrantLock m_runMutex = new ReentrantLock();
    private static final List<BooleanSupplier> m_periodicRunnables = new ArrayList<>();
    private static final List<BooleanSupplier> m_periodicRunnablesToRemove = new ArrayList<>();
    private static final List<Consumer<Command>> m_commandFinishListeners = new ArrayList<>();
    private static final List<Consumer<Command>> m_commandInitializedListeners = new ArrayList<>();
    private static final List<BiConsumer<Command, Optional<Command>>> m_commandInterruptedListeners = new ArrayList<>();

    private static volatile AtomicLong timestamp = new AtomicLong(0);
    private static final AtomicInteger periodicCount = new AtomicInteger(0);

    /**
     * Acts as the robot time source
     * Function to return the time in microseconds.
     */
    private static long getTimestamp() {
        return timestamp.get();
    }

    /**
     * Local callback to run periodic.
     * This is called every 20ms (50Hz) by the robot code.
     * It runs all the periodic runnables that were added to the list.
     * Also increments the timestamp by 20ms to simulate the robot time.
     */
    private static void periodicRunnable() {
        timestamp.getAndAdd(20 * 1000); // 20ms in microseconds
        periodicCount.getAndIncrement();

        m_runMutex.lock();

        try {
            for (BooleanSupplier r : m_periodicRunnables) {
                if (!r.getAsBoolean()) {
                    // If the runnable returns false, it is marked for removal
                    m_periodicRunnablesToRemove.add(r);
                }
            }

            // Remove periodic runnables that were marked for removal
            m_periodicRunnables.removeAll(m_periodicRunnablesToRemove);
            m_periodicRunnablesToRemove.clear();
        } finally {
            m_runMutex.unlock();
        }
    }

    /**
     * Local callback to handle command finishes
     * @param cmd the command that just finished
     */
    private static void onCommandFinish(Command cmd) {
        for (Consumer<Command> c : m_commandFinishListeners) {
            c.accept(cmd);
        }
    }

    /**
     * Local callback to handle command initializes
     * @param cmd the command that just initialized
     */
    private static void onCommandInitialize(Command cmd) {
        for (Consumer<Command> c : m_commandInitializedListeners) {
            c.accept(cmd);
        }
    }

    /**
     * Local callback to handle command interrupt
     * @param cmd the command that just interrupted
     * @param interrupting the command that interrupted it, if any
     */
    private static void onCommandInterrupt(Command cmd, Optional<Command> interrupting) {
        for (BiConsumer<Command, Optional<Command>> c : m_commandInterruptedListeners) {
            c.accept(cmd, interrupting);
        }
    }

    /**
     * Ends the robot and cleans up resources.
     * This should only be called after all tests are done running,
     * since the unit tests reuse the robot instance to save time
     */
    public static void endRobot() {
        if (testRobot != null) {
            testRobot.endCompetition();

            try {
                robotThread.join(5000);
            } catch (InterruptedException e) {
                ConsoleLogger.logError(e);
            }

            testRobot = null;
        }
    }

    /**
     * Creates a stop condition that will return true when the command ends that matches the given predicate.
     * @param commandPredicate a predicate that tests the command to determine if it should stop
     * @return a supplier that returns true when the command ends that matches the predicate
     */
    public static BooleanSupplierEx stopOnCommandEnd(Predicate<Command> commandPredicate) {
        boolean[] shouldStop = {false};
        Consumer<Command> listener = cmd -> {
            if (commandPredicate.test(cmd)) {
                shouldStop[0] = true;
            }
        };
        m_runMutex.lock();
        try {
            m_commandFinishListeners.add(listener);
        } finally {
            m_runMutex.unlock();
        }

        return () -> {
            boolean stop = shouldStop[0];
            if (stop) {
                m_runMutex.lock();
                try {
                    m_commandFinishListeners.remove(listener);
                } finally {
                    m_runMutex.unlock();
                }
            }
            return stop;
        };
    }

    /**
     * Creates a stop condition that will return true when the command initializes that matches the given predicate.
     * @param commandPredicate a predicate that tests the command to determine if it should stop
     * @return a supplier that returns true when the command initializes that matches the predicate
     */
    public static BooleanSupplierEx stopOnCommandInit(Predicate<Command> commandPredicate) {
        boolean[] shouldStop = {false};
        Consumer<Command> listener = cmd -> {
            if (commandPredicate.test(cmd)) {
                shouldStop[0] = true;
            }
        };
        m_runMutex.lock();
        try {
            m_commandInitializedListeners.add(listener);
        } finally {
            m_runMutex.unlock();
        }

        return () -> {
            boolean stop = shouldStop[0];
            if (stop) {
                m_runMutex.lock();
                try {
                    m_commandInitializedListeners.remove(listener);
                } finally {
                    m_runMutex.unlock();
                }
            }
            return stop;
        };
    }

    /**
     * Creates a stop condition that will return true when the command is interrupted that matches the given predicate.
     * @param commandPredicate a predicate that tests the command (and optionally the interrupting command) to determine if it should stop
     * @return a supplier that returns true when the command initializes that matches the predicate
     */
    public static BooleanSupplierEx stopOnCommandInterrupt(
            BiFunction<Command, Optional<Command>, Boolean> commandPredicate) {
        boolean[] shouldStop = {false};

        BiConsumer<Command, Optional<Command>> listener = (cmd, intr) -> {
            if (commandPredicate.apply(cmd, intr)) {
                shouldStop[0] = true;
            }
        };
        m_runMutex.lock();
        try {
            m_commandInterruptedListeners.add(listener);
        } finally {
            m_runMutex.unlock();
        }

        return () -> {
            boolean stop = shouldStop[0];
            if (stop) {
                m_runMutex.lock();
                try {
                    m_commandInterruptedListeners.remove(listener);
                } finally {
                    m_runMutex.unlock();
                }
            }
            return stop;
        };
    }

    /**
     * Creates a stop condition that will return true after a certain number of periodic cycles
     * when the stop condition is met. If the stop condition returns false after being true, it resets the cycle count.
     * @param cycles the number of periodic cycles to wait before stopping, if less than or equal to 0, it will stop immediately
     * @param stopCondition a supplier that returns true when the stop condition is met
     * @return a supplier that returns true when the stop condition is met after the specified cycles
     */
    public static BooleanSupplierEx delayStop(int cycles, BooleanSupplierEx stopCondition) {
        if (stopCondition == null) throw new IllegalArgumentException("stopCondition cannot be null");

        if (cycles <= 0) {
            return stopCondition; // If cycles is less than or equal to 0, return the stop condition directly
        }

        int[] currentCount = {-1};
        return () -> {
            boolean stop = stopCondition.getAsBoolean();
            if (stop) {
                int count = periodicCount.get();
                if (currentCount[0] == -1) {
                    currentCount[0] = count;
                } else if (count >= currentCount[0] + cycles) {
                    return true; // Stop condition met after the specified cycles
                }
            } else {
                currentCount[0] = -1; // Reset if stop condition is not met
            }
            return false;
        };
    }

    /**
     * Runs the robot until the stop condition is met, with a robot configuration and optional periodic runnable.
     * Finally, runs the test function to verify the robot's state.
     * The default timeout is 10 seconds.
     * @param stopCondition a supplier that returns true when the test should stop
     * @param periodic an optional periodic runnable that runs every 20ms (can also be null). return true to call it next periodic, false to stop calling it.
     * @param robotConfig a consumer that configures the robot before the test starts
     * @param testFunction a runnable that runs after the stop condition is met to verify the robot's state
     */
    public static void testUntil(
            BooleanSupplierEx stopCondition,
            BooleanSupplierEx periodic,
            Consumer<Robot> robotConfig,
            Runnable testFunction) {
        testUntil(stopCondition, periodic, robotConfig, testFunction, Seconds.of(10.0));
    }

    /**
     * Runs the robot until the stop condition is met, with a robot configuration and optional periodic runnable.
     * Finally, runs the test function to verify the robot's state.
     * @param stopCondition a supplier that returns true when the test should stop
     * @param periodic an optional periodic boolean supplier that runs every 20ms (can also be null). return true to call it next periodic, false to stop calling it.
     * @param robotConfig a consumer that configures the robot before the test starts
     * @param testFunction a runnable that runs after the stop condition is met to verify the robot's state
     * @param timeout the maximum time to run the test before failing, if less than or equal to 0, no timeout
     */
    public static void testUntil(
            BooleanSupplierEx stopCondition,
            BooleanSupplierEx periodic,
            Consumer<Robot> robotConfig,
            Runnable testFunction,
            Time timeout) {

        if (stopCondition == null) throw new IllegalArgumentException("stopCondition cannot be null");
        if (robotConfig == null) throw new IllegalArgumentException("robotConfig cannot be null");
        if (testFunction == null) throw new IllegalArgumentException("testFunction cannot be null");

        if (testRobot == null) {
            m_runMutex.lock();
            try {
                testRobot = new Robot();

                RobotController.setTimeSource(TestRobot::getTimestamp);
                testRobot.setPeriodicRunnable(TestRobot::periodicRunnable);
                CommandScheduler.getInstance().onCommandInitialize(TestRobot::onCommandInitialize);
                CommandScheduler.getInstance().onCommandInterrupt(TestRobot::onCommandInterrupt);
                CommandScheduler.getInstance().onCommandFinish(TestRobot::onCommandFinish);

                robotThread = new Thread(() -> {
                    Constants.RobotState.setRunningAsUnitTest();
                    RobotBase.startRobot(() -> testRobot);
                });
                robotThread.setDaemon(true);
                robotThread.setName("TestRobot Daemon");
                robotThread.start();
            } finally {
                m_runMutex.unlock();
            }
        }

        m_runMutex.lock();
        try {
            TestRobot.waitForInit();
        } finally {
            m_runMutex.unlock();
        }
        waitPeriodicCycles(1);
        m_runMutex.lock();
        try {
            robotConfig.accept(testRobot);
            if (periodic != null) {
                m_periodicRunnables.add(periodic);
            }
        } finally {
            m_runMutex.unlock();
        }

        long startTime = getTimestamp();
        if (timeout.gt(Seconds.zero())) {
            stopCondition = stopCondition.or(() -> getTimestamp() - startTime >= timeout.in(Microseconds));
        }

        try {
            while (!stopCondition.getAsBoolean()) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    ConsoleLogger.logError(e);
                }
            }
        } finally {
            if (periodic != null) {
                m_runMutex.lock();
                try {
                    m_periodicRunnables.remove(periodic);
                } finally {
                    m_runMutex.unlock();
                }
            }
        }

        testFunction.run();
    }

    /**
     * Runs a runnable after a certain number of periodic cycles.
     * @param cycles the number of periodic cycles to wait before running the runnable
     * @param runnable the runnable to run after the specified number of cycles
     * @throws IllegalArgumentException if cycles is less than or equal to 0, or if runnable is null
     */
    public static void runAfter(int cycles, Runnable runnable) {
        if (runnable == null) throw new IllegalArgumentException("runnable cannot be null");
        if (cycles <= 0) throw new IllegalArgumentException("cycles must be greater than 0");

        m_runMutex.lock();
        try {
            int currentPeriodicCount = periodicCount.get();
            m_periodicRunnables.add(() -> {
                if (currentPeriodicCount + cycles == periodicCount.get()) {
                    runnable.run();
                    return false;
                }

                return true;
            });
        } finally {
            m_runMutex.unlock();
        }
    }

    /**
     * Runs a runnable for a certain number of periodic cycles.
     * @param cycles the number of periodic cycles to run the runnable for
     * @param runnable the runnable to run for the specified number of cycles
     * @throws IllegalArgumentException if cycles is less than or equal to 0, or if runnable is null
     */
    public static void runFor(int cycles, Runnable runnable) {
        if (runnable == null) throw new IllegalArgumentException("runnable cannot be null");
        if (cycles <= 0) throw new IllegalArgumentException("cycles must be greater than 0");

        m_runMutex.lock();
        try {
            int currentPeriodicCount = periodicCount.get();
            m_periodicRunnables.add(() -> {
                if (periodicCount.get() <= currentPeriodicCount + cycles) {
                    runnable.run();
                    return true;
                }

                return false;
            });
        } finally {
            m_runMutex.unlock();
        }
    }

    /**
     * Resets important robot states and waits for the robot to be initialized.
     * Runs before every unit test (call to robotConfig)
     */
    public static void waitForInit() {
        TestControlBridge.getInstance().reset();
        SimulatedArena.getInstance().clearGamePieces();
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.setMatchTime(0);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        while (!testRobot.isInitialized()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        RobotContainer.resetEncoders();
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Waits for a certain number of periodic cycles to pass.
     * Note: this is a busy wait, so it will block the current thread - so DO NOT run on the robot thread (or inside any callback).
     * @param cycles the number of periodic cycles to wait for
     * @throws IllegalArgumentException if cycles is less than or equal to 0
     */
    public static void waitPeriodicCycles(int cycles) {
        if (cycles <= 0) throw new IllegalArgumentException("cycles must be greater than 0");

        int startCount = periodicCount.get();

        while (true) {
            int currentCount = periodicCount.get();

            if (currentCount >= startCount + cycles) {
                break;
            }

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    /**
     * Helper function for shorter access to {@link TestControlBridge#getInstance() TestControlBridge.getInstance()}
     */
    public static TestControlBridge controlBridge() {
        return TestControlBridge.getInstance();
    }
}
