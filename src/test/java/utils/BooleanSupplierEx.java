package utils;

import java.util.function.BooleanSupplier;

@FunctionalInterface
public interface BooleanSupplierEx extends BooleanSupplier {
    default BooleanSupplierEx and(BooleanSupplier other) {
        return () -> this.getAsBoolean() && other.getAsBoolean();
    }

    default BooleanSupplierEx or(BooleanSupplier other) {
        return () -> this.getAsBoolean() || other.getAsBoolean();
    }

    default BooleanSupplierEx negate() {
        return () -> !this.getAsBoolean();
    }
}
