package org.firstinspires.ftc.teamcode.library.control;

import java.util.Objects;

/**
 * Selects a near controller inside an error threshold and a far controller
 * outside it. Both controllers update every cycle to preserve their state.
 */
public class CompositeController<T> implements Controller<T, T, T> {
    private final CompositeConstants<T> constants;
    private T target;
    private Controller<T, T, T> selectedController;

    public CompositeController(CompositeConstants<T> constants) {
        this.constants = Objects.requireNonNull(constants, "constants");
    }

    @Override
    public void update(T state) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(target, "target");

        double[] current = stateArray(state, "stateSupplier result");
        double[] desired = stateArray(target, "target state");
        double[] threshold = constants.getThreshold();
        if (threshold == null) {
            throw new IllegalStateException(
                    "Threshold not configured; call constants.setThreshold(...) before update().");
        }
        validateDimensions(current, desired, threshold);

        constants.getNearController().update(state);
        constants.getFarController().update(state);
        selectedController = isWithinThreshold(current, desired, threshold)
                ? constants.getNearController()
                : constants.getFarController();
    }

    @Override
    public void reset() {
        constants.getNearController().reset();
        constants.getFarController().reset();
        selectedController = null;
    }

    @Override
    public void setTarget(T target) {
        this.target = Objects.requireNonNull(target, "target");
        constants.getNearController().setTarget(target);
        constants.getFarController().setTarget(target);
    }

    @Override
    public T getTarget() {
        return target;
    }

    @Override
    public T getOutput() {
        return selectedController == null ? null : selectedController.getOutput();
    }

    @Override
    public boolean atTarget() {
        return selectedController != null && selectedController.atTarget();
    }

    public CompositeConstants<T> getConstants() {
        return constants;
    }

    public boolean isUsingNearController() {
        return selectedController == constants.getNearController();
    }

    private double[] stateArray(T value, String name) {
        return Objects.requireNonNull(constants.getStateSupplier().apply(value), name);
    }

    private static void validateDimensions(double[] current, double[] desired, double[] threshold) {
        if (current.length == 0) {
            throw new IllegalArgumentException("State values must not be empty.");
        }
        if (desired.length != current.length || threshold.length != current.length) {
            throw new IllegalArgumentException(
                    "State, target, and threshold dimensions must match.");
        }
    }

    private static boolean isWithinThreshold(double[] current, double[] desired, double[] threshold) {
        for (int index = 0; index < current.length; index++) {
            if (Math.abs(desired[index] - current[index]) > threshold[index]) {
                return false;
            }
        }
        return true;
    }
}
