package org.firstinspires.ftc.teamcode.library.control;

import java.util.Objects;
import java.util.function.Function;

/**
 * Configuration for a {@link BangBangController}. Outputs use the same value shape as
 * state and target: the supplied conversion functions map that shape to components.
 */
public class BangBangConstants<T> {
    private T belowOutput;
    private T aboveOutput;
    private T sameOutput;
    private double[] tolerance;
    private Function<T, double[]> stateSupplier;
    private Function<double[], T> inverseStateSupplier;

    public BangBangConstants(
            Function<T, double[]> stateSupplier,
            Function<double[], T> inverseStateSupplier) {
        this.stateSupplier = stateSupplier;
        this.inverseStateSupplier = inverseStateSupplier;
    }

    public BangBangConstants<T> setOutputs(T belowOutput, T aboveOutput, T sameOutput) {
        this.belowOutput = Objects.requireNonNull(belowOutput, "belowOutput");
        this.aboveOutput = Objects.requireNonNull(aboveOutput, "aboveOutput");
        this.sameOutput = Objects.requireNonNull(sameOutput, "sameOutput");
        return this;
    }

    public BangBangConstants<T> setTolerance(double[] tolerance) {
        if (tolerance == null || tolerance.length == 0) {
            throw new IllegalArgumentException("tolerance must be a non-empty array.");
        }
        for (double component : tolerance) {
            if (component < 0.0 || Double.isNaN(component)) {
                throw new IllegalArgumentException("tolerance values must be non-negative numbers.");
            }
        }
        this.tolerance = tolerance.clone();
        return this;
    }

    public BangBangConstants<T> setTolerance(double tolerance) {
        if (tolerance < 0.0 || Double.isNaN(tolerance)) {
            throw new IllegalArgumentException("tolerance must be a non-negative number.");
        }
        return setTolerance(fillArray(resolveStateLength(), tolerance));
    }

    public void setStateSupplier(Function<T, double[]> stateSupplier) {
        this.stateSupplier = Objects.requireNonNull(stateSupplier, "stateSupplier");
    }

    public void setInverseStateSupplier(Function<double[], T> inverseStateSupplier) {
        this.inverseStateSupplier = Objects.requireNonNull(inverseStateSupplier, "inverseStateSupplier");
    }

    public T getBelowOutput() {
        return belowOutput;
    }

    public T getAboveOutput() {
        return aboveOutput;
    }

    public T getSameOutput() {
        return sameOutput;
    }

    public double[] getTolerance() {
        return tolerance == null ? null : tolerance.clone();
    }

    Function<T, double[]> getStateSupplier() {
        return stateSupplier;
    }

    Function<double[], T> getInverseStateSupplier() {
        return inverseStateSupplier;
    }

    boolean hasOutputs() {
        return belowOutput != null && aboveOutput != null && sameOutput != null;
    }

    private int resolveStateLength() {
        if (hasOutputs() && stateSupplier != null) {
            double[] output = Objects.requireNonNull(
                    stateSupplier.apply(belowOutput), "stateSupplier result");
            if (output.length == 0) {
                throw new IllegalArgumentException("State output must not be empty.");
            }
            return output.length;
        }
        throw new IllegalStateException(
                "Unable to determine the bang-bang state dimension. Configure an array tolerance "
                        + "or configure outputs with a state supplier first.");
    }

    private static double[] fillArray(int length, double value) {
        double[] values = new double[length];
        for (int index = 0; index < length; index++) {
            values[index] = value;
        }
        return values;
    }
}
