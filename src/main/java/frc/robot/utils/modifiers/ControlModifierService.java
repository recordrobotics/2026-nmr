package frc.robot.utils.modifiers;

import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

@SuppressWarnings("java:S6548") // Singleton: There is only one service
public final class ControlModifierService {

    private static ControlModifierService instance;

    public record ControlModifier(IDrivetrainControlModifier modifier, IDrivetrainControlModifier.Priority priority)
            implements Comparable<ControlModifier> {
        @Override
        public int compareTo(ControlModifier o) {
            int comp = Integer.compare(this.priority.getValue(), o.priority.getValue());
            if (comp == 0) {
                // If priorities are the same, compare by instance hashcode to avoid equality issues
                return Integer.compare(System.identityHashCode(this.modifier), System.identityHashCode(o.modifier));
            } else {
                return comp;
            }
        }
    }

    private static final ReentrantLock modifierMutex = new ReentrantLock();
    private SortedSet<ControlModifier> modifiers = new TreeSet<>();

    private ControlModifierService() {}

    public static synchronized ControlModifierService getInstance() {
        if (instance == null) {
            instance = new ControlModifierService();
        }
        return instance;
    }

    public SortedSet<ControlModifier> getModifiers() {
        SortedSet<ControlModifier> localCopy;
        modifierMutex.lock();
        try {
            // Return a copy to avoid concurrent modification issues
            localCopy = new TreeSet<>(modifiers);
        } finally {
            modifierMutex.unlock();
        }
        return localCopy;
    }

    public <T extends IDrivetrainControlModifier> T createModifier(
            Supplier<T> modifier, IDrivetrainControlModifier.Priority priority) {
        T modifierInstance = modifier.get();
        modifierMutex.lock();
        try {
            modifiers.add(new ControlModifier(modifierInstance, priority));
        } finally {
            modifierMutex.unlock();
        }
        return modifierInstance;
    }
}
