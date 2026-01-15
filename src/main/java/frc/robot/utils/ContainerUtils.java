package frc.robot.utils;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSortedMap;
import java.util.Map;

public final class ContainerUtils {

    private ContainerUtils() {}

    public static <K, V> ImmutableMap<V, K> reverseMap(Map<K, V> map) {
        ImmutableMap.Builder<V, K> builder = ImmutableMap.builder();
        for (Map.Entry<K, V> entry : map.entrySet()) {
            builder.put(entry.getValue(), entry.getKey());
        }
        return builder.build();
    }

    public static <K, V extends Comparable<? super V>> ImmutableSortedMap<V, K> reverseMap(
            ImmutableSortedMap<K, V> map) {
        ImmutableSortedMap.Builder<V, K> builder = ImmutableSortedMap.<V, K>naturalOrder();
        for (Map.Entry<K, V> entry : map.entrySet()) {
            builder.put(entry.getValue(), entry.getKey());
        }
        return builder.build();
    }

    public static <T> boolean contains(T[] array, T value) {
        for (T element : array) {
            if (element.equals(value)) {
                return true;
            }
        }
        return false;
    }

    public static boolean contains(int[] array, int value) {
        for (int element : array) {
            if (element == value) {
                return true;
            }
        }
        return false;
    }
}
