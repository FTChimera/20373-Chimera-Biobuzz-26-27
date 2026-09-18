package org.firstinspires.ftc.teamcode.library.control;

import java.util.Objects;
import java.util.function.Function;

/**
 * A component-wise three-state controller. Error is defined as {@code target - state}:
 * a positive error outside tolerance selects the below-target output, a negative error
 * outside tolerance selects the above-target output, and an in-tolerance error selects
 * the same-target output.
 */
public class BangBangController<T> implements Controller<T, T, T> {
    private BangBangConstants<T> constants;
    private T target;
    private T output;
    private double[] lastError;
    private boolean hasUpdate;

    public BangBangController(BangBangConstants<T> constants) {
        setConstants(constants);
    }

    public final void setConstants(BangBangConstants<T> constants) {
        this.constants = Objects.requireNonNull(constants, "constants");
        reset();
    }

    public BangBangConstants<T> getConstants() {
        return constants;
    }

    public void setStateSupplier(Function<T, double[]> stateSupplier) {
        constants.setStateSupplier(stateSupplier);
    }

    public void setInverseStateSupplier(Function<double[], T> inverseStateSupplier) {
        constants.setInverseStateSupplier(inverseStateSupplier);
    }

    @Override
    public void update(T state) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(target, "target");

        Function<T, double[]> stateSupplier = constants.getStateSupplier();
        Function<double[], T> inverseStateSupplier = constants.getInverseStateSupplier();
        if (stateSupplier == null || inverseStateSupplier == null) {
            throw new IllegalStateException(
                    "State suppliers not configured; call setStateSupplier(...) and "
                            + "setInverseStateSupplier(...) before update().");
        }
        if (!constants.hasOutputs()) {
            throw new IllegalStateException(
                    "Bang-bang outputs not configured; call constants.setOutputs(...) before update().");
        }

        double[] tolerance = constants.getTolerance();
        if (tolerance == null) {
            throw new IllegalStateException(
                    "Tolerance not configured; call constants.setTolerance(...) before update().");
        }

        double[] currentState = stateArray(stateSupplier, state, "stateSupplier result");
        double[] targetState = stateArray(stateSupplier, target, "target state");
        validateLength(targetState, "target state", currentState.length);
        validateLength(tolerance, "tolerance", currentState.length);

        double[] belowOutput = stateArray(stateSupplier, constants.getBelowOutput(), "below-target output");
        double[] aboveOutput = stateArray(stateSupplier, constants.getAboveOutput(), "above-target output");
        double[] sameOutput = stateArray(stateSupplier, constants.getSameOutput(), "same-target output");
        validateLength(belowOutput, "below-target output", currentState.length);
        validateLength(aboveOutput, "above-target output", currentState.length);
        validateLength(sameOutput, "same-target output", currentState.length);

        double[] error = new double[currentState.length];
        double[] selectedOutput = new double[currentState.length];
        for (int index = 0; index < currentState.length; index++) {
            error[index] = targetState[index] - currentState[index];
            if (error[index] > tolerance[index]) {
                selectedOutput[index] = belowOutput[index];
            } else if (error[index] < -tolerance[index]) {
                selectedOutput[index] = aboveOutput[index];
            } else {
                selectedOutput[index] = sameOutput[index];
            }
        }

        output = Objects.requireNonNull(
                inverseStateSupplier.apply(selectedOutput), "inverseStateSupplier result");
        lastError = error;
        hasUpdate = true;
    }

    @Override
    public void reset() {
        output = null;
        lastError = null;
        hasUpdate = false;
    }

    @Override
    public void setTarget(T target) {
        this.target = Objects.requireNonNull(target, "target");
    }

    @Override
    public T getTarget() {
        return target;
    }

    public T getLastError() {
        if (!hasUpdate) {
            throw new IllegalStateException("No update has been performed yet; call update() first.");
        }
        Function<double[], T> inverseStateSupplier = constants.getInverseStateSupplier();
        if (inverseStateSupplier == null) {
            throw new IllegalStateException("Inverse state supplier is not configured.");
        }
        return Objects.requireNonNull(
                inverseStateSupplier.apply(lastError.clone()), "inverseStateSupplier result");
    }

    @Override
    public T getOutput() {
        return output;
    }

    @Override
    public boolean atTarget() {
        if (!hasUpdate) {
            return false;
        }

        double[] tolerance = constants.getTolerance();
        return tolerance != null && isAtTarget(lastError, tolerance);
    }

    private static <T> double[] stateArray(Function<T, double[]> stateSupplier, T value, String name) {
        return Objects.requireNonNull(stateSupplier.apply(value), name);
    }

    private static void validateLength(double[] values, String name, int expectedLength) {
        if (values.length == 0) {
            throw new IllegalArgumentException(name + " must not be empty.");
        }
        if (values.length != expectedLength) {
            throw new IllegalArgumentException(
                    name + " must have length " + expectedLength + " but had " + values.length + '.');
        }
    }

    private static boolean isAtTarget(double[] error, double[] tolerance) {
        if (error.length != tolerance.length) {
            return false;
        }
        for (int index = 0; index < error.length; index++) {
            if (Math.abs(error[index]) > tolerance[index]) {
                return false;
            }
        }
        return true;
    }
}
