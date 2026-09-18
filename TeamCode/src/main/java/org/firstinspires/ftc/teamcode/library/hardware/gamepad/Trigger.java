package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

/**
 * Represents one analog trigger (left or right) with threshold-based digital
 * detection, edge detection, and toggle support.
 *
 * <pre>
 * Trigger lt = Trigger.left(0.5);   // threshold = 0.5
 * // in loop:
 * lt.update(gamepad1);
 * double raw = lt.getValue();
 * if (lt.wasJustPressed()) { ... }
 * </pre>
 */
public class Trigger extends GpCommand {
    private static final DoubleUnaryOperator LINEAR_OUTPUT = value -> value;

    /** Which physical trigger this represents. */
    public enum Side { LEFT, RIGHT }

    public static final Trigger LEFT = new Trigger(Side.LEFT, 0.5);
    public static final Trigger RIGHT = new Trigger(Side.RIGHT, 0.5);

    private final Side side;
    private double threshold;
    private DoubleUnaryOperator outputTransform = LINEAR_OUTPUT;

    private double rawValue;
    private boolean current;
    private boolean previous;
    private boolean toggleState;

    /**
     * Creates a trigger reader.
     * @param side      Which trigger (LEFT or RIGHT)
     * @param threshold Digital threshold in (0, 1]. Values ≥ threshold → "pressed".
     */
    public Trigger(Side side, double threshold) {
        super(TYPE.HYBRID, side == Side.LEFT ? "LEFT_TRIGGER" : "RIGHT_TRIGGER");
        this.side = side;
        this.threshold = Math.max(0.01, Math.min(threshold, 1.0));
    }

    /** Convenience: creates a LEFT trigger with the given threshold. */
    public static Trigger left(double threshold) { return new Trigger(Side.LEFT, threshold); }

    /** Convenience: creates a RIGHT trigger with the given threshold. */
    public static Trigger right(double threshold) { return new Trigger(Side.RIGHT, threshold); }

    /** Convenience: creates a LEFT trigger with default threshold 0.5. */
    public static Trigger left() { return left(0.5); }

    /** Convenience: creates a RIGHT trigger with default threshold 0.5. */
    public static Trigger right() { return right(0.5); }

    /**
     * Snapshots the trigger value from the gamepad.
     * Must be called once per loop iteration.
     * @param gamepad The raw FTC Gamepad
     */
    public void update(Gamepad gamepad) {
        previous = current;
        rawValue = (side == Side.LEFT) ? gamepad.left_trigger : gamepad.right_trigger;
        current = rawValue >= threshold;
        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    // ==================== QUERIES ====================

    /** @return Which physical trigger this is */
    public Side getSide() { return side; }

    @Override
    public GpCommand copy() {
        Trigger copy = new Trigger(side, threshold);
        copy.setOutputTransform(outputTransform);
        return copy;
    }

    /** @return Analog value after the configured output transform. */
    public double getValue() { return outputTransform.applyAsDouble(rawValue); }

    /** @return Raw analog value [0, 1], before output transformation. */
    public double getRawValue() { return rawValue; }

    /** @return true if trigger value ≥ threshold */
    public boolean isPressed() { return current; }

    /** @return true on the frame the trigger first exceeded the threshold (rising edge) */
    public boolean wasJustPressed() { return current && !previous; }

    /** @return true on the frame the trigger dropped below the threshold (falling edge) */
    public boolean wasJustReleased() { return !current && previous; }

    /** @return Toggle that flips on every press */
    public boolean getToggle() { return toggleState; }

    /** Manually overrides the toggle state. */
    public void setToggle(boolean state) { this.toggleState = state; }

    // ==================== CONFIGURATION ====================

    /**
     * Sets the digital threshold.
     * @param threshold Value in (0, 1]. Default 0.5.
     */
    public void setThreshold(double threshold) {
        this.threshold = Math.max(0.01, Math.min(threshold, 1.0));
    }

    /** @return The current digital threshold */
    public double getThreshold() { return threshold; }

    /**
     * Sets the transformation applied once when reading the analog output.
     * Digital threshold and edge detection continue to use the raw trigger value.
     * @param outputTransform Function applied to raw trigger input
     */
    public void setOutputTransform(DoubleUnaryOperator outputTransform) {
        this.outputTransform = Objects.requireNonNull(outputTransform, "outputTransform must not be null");
    }

    /** @return The function applied to the raw analog trigger output. */
    public DoubleUnaryOperator getOutputTransform() { return outputTransform; }
}
