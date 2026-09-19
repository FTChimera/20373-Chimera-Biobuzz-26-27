package org.firstinspires.ftc.teamcode.library.control;

import java.util.Objects;
import java.util.function.Function;

/**
 * Configuration for a {@link CompositeController}. Controller one is selected
 * when every absolute error component is within its corresponding threshold.
 */
public class CompositeConstants<T> {
    private final Controller<T, T, T> nearController;
    private final Controller<T, T, T> farController;
    private final Function<T, double[]> stateSupplier;
    private double[] threshold;

    public CompositeConstants(
            Controller<T, T, T> nearController,
            Controller<T, T, T> farController,
            Function<T, double[]> stateSupplier) {
        this.nearController = Objects.requireNonNull(nearController, "nearController");
        this.farController = Objects.requireNonNull(farController, "farController");
        this.stateSupplier = Objects.requireNonNull(stateSupplier, "stateSupplier");
    }

    public CompositeConstants<T> setThreshold(double[] threshold) {
        if (threshold == null || threshold.length == 0) {
            throw new IllegalArgumentException("threshold must be a non-empty array.");
        }
        for (double component : threshold) {
            if (component < 0.0 || !Double.isFinite(component)) {
                throw new IllegalArgumentException("threshold values must be finite, non-negative numbers.");
            }
        }
        this.threshold = threshold.clone();
        return this;
    }

    public CompositeConstants<T> setThreshold(double threshold) {
        if (threshold < 0.0 || !Double.isFinite(threshold)) {
            throw new IllegalArgumentException("threshold must be a finite, non-negative number.");
        }
        return setThreshold(new double[]{threshold});
    }

    public Controller<T, T, T> getNearController() {
        return nearController;
    }

    public Controller<T, T, T> getFarController() {
        return farController;
    }

    public Function<T, double[]> getStateSupplier() {
        return stateSupplier;
    }

    public double[] getThreshold() {
        return threshold == null ? null : threshold.clone();
    }
}
