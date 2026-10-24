package org.firstinspires.ftc.teamcode.util.vars;

import java.util.Objects;

@FunctionalInterface
public interface Validator<T> {
    boolean validate(T value);
    static Validator<Double> positive() {
        return (v) -> v > (double)0.0F;
    }

    static Validator<Double> nonnegative() {
        return (v) -> v >= (double)0.0F;
    }

    static Validator<Double> negative() {
        return (v) -> v < (double)0.0F;
    }

    static Validator<Double> nonpositive() {
        return (v) -> v <= (double)0.0F;
    }

    static <T> Validator<T> nonnull() {
        return Objects::nonNull;
    }

    static Validator<Double> inRange(double min, double max) {
        return (v) -> v >= min && v <= max;
    }

    static Validator<Double> nonZero() {
        return (v) -> v != 0.0;
    }
}
