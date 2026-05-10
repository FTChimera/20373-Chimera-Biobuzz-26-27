package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

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
public class Trigger {

    /** Which physical trigger this represents. */
    public enum Side { LEFT, RIGHT }

    private final Side side;
    private double threshold;

    private double value;
    private boolean current;
    private boolean previous;
    private boolean toggleState;

    /**
     * Creates a trigger reader.
     * @param side      Which trigger (LEFT or RIGHT)
     * @param threshold Digital threshold in (0, 1]. Values ≥ threshold → "pressed".
     */
    public Trigger(Side side, double threshold) {
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
        value = (side == Side.LEFT) ? gamepad.left_trigger : gamepad.right_trigger;
        current = value >= threshold;
        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    // ==================== QUERIES ====================

    /** @return Which physical trigger this is */
    public Side getSide() { return side; }

    /** @return Raw analog value [0, 1] */
    public double getValue() { return value; }

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
}
