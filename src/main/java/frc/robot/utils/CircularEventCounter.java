package frc.robot.utils;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * A utility class for counting events that occur within a specified time window using a circular buffer.
 */
public class CircularEventCounter {

    private final CircularBuffer<Double> eventTimestamps;
    private double timeWindowSeconds;

    /**
     * Creates a CircularEventCounter with the specified maximum count and time window.
     * @param maxCount The maximum number of events to track in the circular buffer.
     * @param timeWindowSeconds The time window in seconds for counting events. Only events that occurred within this time window will be counted.
     */
    public CircularEventCounter(int maxCount, double timeWindowSeconds) {
        this.eventTimestamps = new CircularBuffer<>(maxCount);
        this.timeWindowSeconds = timeWindowSeconds;
    }

    /**
     * Records an event by adding the current timestamp to the circular buffer.
     */
    public void recordEvent() {
        eventTimestamps.addFirst(Timer.getTimestamp());
    }

    /**
     * Returns the count of events that occurred within the specified time window.
     * @return The number of events within the time window.
     */
    public int getEventCount() {
        double currentTime = Timer.getTimestamp();
        int count = 0;
        for (int i = 0; i < eventTimestamps.size(); i++) {
            if (currentTime - eventTimestamps.get(i) <= timeWindowSeconds) {
                count++;
            } else {
                break; // Since timestamps are in order, we can stop checking once we find an old event
            }
        }
        return count;
    }

    /**
     * Clears all recorded events from the circular buffer.
     */
    public void clear() {
        eventTimestamps.clear();
    }

    /**
     * Resizes the circular buffer to a new maximum count. This will clear all existing events.
     * @param maxCount The new maximum number of events to track in the circular buffer.
     */
    public void resize(int maxCount) {
        eventTimestamps.resize(maxCount);
    }

    /**
     * Sets the time window for counting events. Only events that occurred within this time window will be counted.
     * @param timeWindowSeconds The new time window in seconds for counting events.
     */
    public void setTimeWindowSeconds(double timeWindowSeconds) {
        this.timeWindowSeconds = timeWindowSeconds;
    }

    /**
     * Returns the current time window in seconds for counting events.
     * @return The time window in seconds.
     */
    public double getTimeWindowSeconds() {
        return timeWindowSeconds;
    }
}
