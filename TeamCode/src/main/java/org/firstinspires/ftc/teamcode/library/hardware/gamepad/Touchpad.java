package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Reads the PS4/PS5 touchpad state, providing finger positions, edge detection
 * for finger contact, and the touchpad button press. Supports up to 2 simultaneous fingers.
 *
 * <p>Coordinates range from -1.0 to 1.0 with (0, 0) at the center of the touchpad.
 * Positive X = right, positive Y = up.
 *
 * <pre>
 * Touchpad tp = new Touchpad();
 * // in loop:
 * tp.update(gamepad1);
 * for (Touchpad.TouchPoint p : tp.getFingers()) {
 *     telemetry.addData("Finger", "x=%.2f y=%.2f", p.x, p.y);
 * }
 * if (tp.wasButtonJustPressed()) { ... }
 * </pre>
 */
public class Touchpad extends GpCommand {
    public static final Touchpad MAIN = new Touchpad();

    // ==================== TOUCHPOINT ====================

    /**
     * Immutable snapshot of a single finger position on the touchpad.
     */
    public static class TouchPoint {
        /** X coordinate, range [-1, 1]. Positive = right. */
        public final double x;
        /** Y coordinate, range [-1, 1]. Positive = up. */
        public final double y;

        public TouchPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /** @return Euclidean distance from the center of the touchpad */
        public double distanceFromCenter() {
            return Math.hypot(x, y);
        }

        /** @return Angle from center in radians (atan2(y, x)). 0 = right, π/2 = up. */
        public double angle() {
            return Math.atan2(y, x);
        }

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "TouchPoint(%.2f, %.2f)", x, y);
        }
    }

    // ==================== FIELDS ====================

    // Finger 1
    private boolean curFinger1;
    private boolean prevFinger1;
    private double finger1X;
    private double finger1Y;

    // Finger 2
    private boolean curFinger2;
    private boolean prevFinger2;
    private double finger2X;
    private double finger2Y;

    // Touchpad button (physical click)
    private boolean curButton;
    private boolean prevButton;
    private boolean buttonToggle;

    // ==================== UPDATE ====================

    /**
     * Snapshots all touchpad state from the gamepad.
     * Must be called once per loop iteration.
     */
    public Touchpad() {
        super(TYPE.HYBRID, "TOUCHPAD");
    }

    public void update(Gamepad gamepad) {
        // Shift current → previous
        prevFinger1 = curFinger1;
        prevFinger2 = curFinger2;
        prevButton = curButton;

        // Finger 1
        curFinger1 = gamepad.touchpad_finger_1;
        if (curFinger1) {
            finger1X = gamepad.touchpad_finger_1_x;
            finger1Y = gamepad.touchpad_finger_1_y;
        }

        // Finger 2
        curFinger2 = gamepad.touchpad_finger_2;
        if (curFinger2) {
            finger2X = gamepad.touchpad_finger_2_x;
            finger2Y = gamepad.touchpad_finger_2_y;
        }

        // Button
        curButton = gamepad.touchpad;
        if (wasButtonJustPressed()) {
            buttonToggle = !buttonToggle;
        }
    }

    // ==================== FINGER LIST ====================

    /**
     * Returns an unmodifiable list of all active finger positions (0, 1, or 2 elements).
     * @return List of {@link TouchPoint}s
     */
    public List<TouchPoint> getFingers() {
        if (!curFinger1 && !curFinger2) {
            return Collections.emptyList();
        }
        List<TouchPoint> out = new ArrayList<>(2);
        if (curFinger1) out.add(new TouchPoint(finger1X, finger1Y));
        if (curFinger2) out.add(new TouchPoint(finger2X, finger2Y));
        return Collections.unmodifiableList(out);
    }

    // ==================== INDIVIDUAL FINGER QUERIES ====================

    /**
     * Gets finger 1 position, or null if not touching.
     * @return TouchPoint for finger 1, or null
     */
    public TouchPoint getFinger1() {
        return curFinger1 ? new TouchPoint(finger1X, finger1Y) : null;
    }

    /**
     * Gets finger 2 position, or null if not touching.
     * @return TouchPoint for finger 2, or null
     */
    public TouchPoint getFinger2() {
        return curFinger2 ? new TouchPoint(finger2X, finger2Y) : null;
    }

    /** @return true if finger 1 is on the touchpad */
    public boolean isFinger1Down() { return curFinger1; }

    /** @return true if finger 2 is on the touchpad */
    public boolean isFinger2Down() { return curFinger2; }

    /** @return true if at least one finger is on the touchpad */
    public boolean isTouched() { return curFinger1 || curFinger2; }

    /** @return The number of fingers currently on the touchpad (0, 1, or 2) */
    public int getFingerCount() { return (curFinger1 ? 1 : 0) + (curFinger2 ? 1 : 0); }

    @Override
    public GpCommand copy() {
        return new Touchpad();
    }

    @Override
    public double getValue() {
        return getFingerCount();
    }

    // ==================== FINGER EDGE DETECTION ====================

    /** @return true on the frame finger 1 first contacted the touchpad */
    public boolean wasFinger1JustPressed() { return curFinger1 && !prevFinger1; }

    /** @return true on the frame finger 1 left the touchpad */
    public boolean wasFinger1JustReleased() { return !curFinger1 && prevFinger1; }

    /** @return true on the frame finger 2 first contacted the touchpad */
    public boolean wasFinger2JustPressed() { return curFinger2 && !prevFinger2; }

    /** @return true on the frame finger 2 left the touchpad */
    public boolean wasFinger2JustReleased() { return !curFinger2 && prevFinger2; }

    // ==================== BUTTON (PHYSICAL CLICK) ====================

    /** @return true if the touchpad button is currently pressed */
    public boolean isButtonPressed() { return curButton; }

    /** @return true on the frame the touchpad button was first pressed */
    public boolean wasButtonJustPressed() { return curButton && !prevButton; }

    /** @return true on the frame the touchpad button was released */
    public boolean wasButtonJustReleased() { return !curButton && prevButton; }

    /** @return Toggle that flips on every touchpad button press */
    public boolean getButtonToggle() { return buttonToggle; }

    /** Manually overrides the button toggle state. */
    public void setButtonToggle(boolean state) { this.buttonToggle = state; }
}
