package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Similar to {@link com.ctre.phoenix6.StatusSignalCollection}, but allows for different CANBus networks to be added which it divides internally
 */
public class ExtendedStatusSignalCollection {

    private final Map<String, List<BaseStatusSignal>> signals = new HashMap<>();

    private final Field deviceIdentifierField;

    public ExtendedStatusSignalCollection() {
        try {
            deviceIdentifierField = BaseStatusSignal.class.getDeclaredField("deviceIdentifier");
            deviceIdentifierField.setAccessible(true);
        } catch (NoSuchFieldException | SecurityException e) {
            ConsoleLogger.logError("Failed to access deviceIdentifier field in BaseStatusSignal", e);
            throw new RuntimeException(e);
        }
    }

    public void add(String name, BaseStatusSignal signal) {
        signals.computeIfAbsent(name, k -> new ArrayList<>()).add(signal);
    }

    public void add(BaseStatusSignal signal) {
        DeviceIdentifier deviceIdentifier;
        try {
            deviceIdentifier = (DeviceIdentifier) deviceIdentifierField.get(signal);
            add(deviceIdentifier.getNetwork().getName(), signal);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            ConsoleLogger.logError("Failed to get device identifier from signal", e);
            throw new RuntimeException(e);
        }
    }

    public void addAll(BaseStatusSignal... signals) {
        for (BaseStatusSignal signal : signals) {
            add(signal);
        }
    }

    public int getDistinctNetworkCount() {
        return signals.size();
    }

    /**
     * Calls {@link BaseStatusSignal#refreshAll()} for every CANBus network separately
     */
    public void refreshAll() {
        for (List<BaseStatusSignal> signalList : signals.values()) {
            BaseStatusSignal.refreshAll(signalList);
        }
    }
}
