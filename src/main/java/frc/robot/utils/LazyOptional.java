package frc.robot.utils;

import java.util.Optional;
import java.util.function.Supplier;

@FunctionalInterface
public interface LazyOptional<T> extends Supplier<Optional<T>> {
    public default LazyOptional<T> orElse(Supplier<Optional<T>> other) {
        return () -> this.get().or(other);
    }
    public default Supplier<T> orElseGet(Supplier<T> other) {
        return () -> this.get().orElseGet(other);
    }
}
