package frc.robot.utils.wrappers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.concurrent.Semaphore;

public class SafeAlert implements AutoCloseable {

    private static final Semaphore alertSemaphore = new Semaphore(1);

    private final Alert alert;

    /**
     * Creates a new alert in the default group - "Alerts". If this is the first to be instantiated,
     * the appropriate entries will be added to NetworkTables.
     *
     * @param text Text to be displayed when the alert is active.
     * @param type Alert urgency level.
     */
    public SafeAlert(String text, AlertType type) {
        tryAcquire();
        try {
            alert = new Alert("Alerts", text, type);
        } finally {
            alertSemaphore.release();
        }
    }

    /**
     * Creates a new alert. If this is the first to be instantiated in its group, the appropriate
     * entries will be added to NetworkTables.
     *
     * @param group Group identifier, used as the entry name in NetworkTables.
     * @param text Text to be displayed when the alert is active.
     * @param type Alert urgency level.
     */
    public SafeAlert(String group, String text, AlertType type) {
        tryAcquire();
        try {
            alert = new Alert(group, text, type);
        } finally {
            alertSemaphore.release();
        }
    }

    /**
     * Sets whether the alert should currently be displayed. This method can be safely called
     * periodically.
     *
     * @param active Whether to display the alert.
     */
    public synchronized void set(boolean active) {
        tryAcquire();
        try {
            alert.set(active);
        } finally {
            alertSemaphore.release();
        }
    }

    /**
     * Gets whether the alert is active.
     *
     * @return whether the alert is active.
     */
    public synchronized boolean get() {
        return alert.get();
    }

    /**
     * Updates current alert text. Use this method to dynamically change the displayed alert, such as
     * including more details about the detected problem.
     *
     * @param text Text to be displayed when the alert is active.
     */
    public synchronized void setText(String text) {
        tryAcquire();
        try {
            alert.setText(text);
        } finally {
            alertSemaphore.release();
        }
    }

    /**
     * Gets the current alert text.
     *
     * @return the current text.
     */
    public synchronized String getText() {
        return alert.getText();
    }

    /**
     * Get the type of this alert.
     *
     * @return the type
     */
    public AlertType getType() {
        return alert.getType();
    }

    @Override
    public synchronized void close() {
        tryAcquire();
        try {
            alert.close();
        } finally {
            alertSemaphore.release();
        }
    }

    private static void tryAcquire() {
        try {
            alertSemaphore.acquire();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException("Interrupted while acquiring alert semaphore", e);
        }
    }
}
