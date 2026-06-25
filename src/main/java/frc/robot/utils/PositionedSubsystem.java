package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.WeakHashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public interface PositionedSubsystem {

    enum PositionStatus {
        /**
         * The position is unknown, possible at the start position
         */
        UNKNOWN,
        /**
         * The position is accurate and known
         */
        KNOWN,
        /**
         * The position could not be determined due to a mechanical issue.
         * (For example elevator hit physical stop above limit switch)
         */
        RESETTING_FAULT,
        /**
         * The position could not be determined due to a sensor fault.
         * (For example encoder or sensor disconnected)
         */
        SENSOR_FAULT
    }

    void resetToStartPosition();

    PositionStatus getPositionStatus();

    void setOverrideKnown(boolean overrideKnown);

    class PositionedSubsystemManager {

        private static PositionedSubsystemManager instance;

        private record Entry(
                SafeAlert warningAlert, SafeAlert errorAlert, String name, LoggedNetworkBoolean overrideKnown) {

            private void setWarning(String errorMessage) {
                warningAlert.setText(errorMessage);
                warningAlert.set(true);
                errorAlert.set(false);
            }

            private void setError(String errorMessage) {
                errorAlert.setText(errorMessage);
                warningAlert.set(false);
                errorAlert.set(true);
            }

            private void clear() {
                warningAlert.set(false);
                errorAlert.set(false);
            }
        }

        private final WeakHashMap<PositionedSubsystem, Entry> subsystems = new WeakHashMap<>();

        public static PositionedSubsystemManager getInstance() {
            if (instance == null) {
                instance = new PositionedSubsystemManager();
            }
            return instance;
        }

        public void registerSubsystem(PositionedSubsystem subsystem) {
            String name = subsystem.getClass().getSimpleName();
            subsystems.put(
                    subsystem,
                    new Entry(
                            new SafeAlert("", AlertType.kWarning),
                            new SafeAlert("", AlertType.kError),
                            name,
                            new LoggedNetworkBoolean("Positioned/" + name + "/OverrideKnown", false)));
        }

        private static void updateEntry(PositionedSubsystem subsystem, Entry entry) {
            subsystem.setOverrideKnown(entry.overrideKnown().get());

            if (entry.overrideKnown().get()) {
                entry.setWarning(entry.name() + " override known");
            } else {
                PositionStatus status = subsystem.getPositionStatus();
                switch (status) {
                    case UNKNOWN -> entry.setWarning(entry.name() + " unknown position");
                    case KNOWN -> entry.clear();
                    case RESETTING_FAULT -> entry.setError(entry.name() + " resetting fault");
                    case SENSOR_FAULT -> entry.setError(entry.name() + " sensor fault");
                }
            }
        }

        public void update() {
            subsystems.forEach(PositionedSubsystemManager::updateEntry);
        }

        public void resetAll() {
            subsystems.keySet().forEach(PositionedSubsystem::resetToStartPosition);
        }
    }
}
