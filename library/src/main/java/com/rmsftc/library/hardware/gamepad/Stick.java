package com.rmsftc.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleUnaryOperator;

/**
 * Represents one analog stick (left or right) with configurable deadzone and
 * response curve. Y-axis is <b>negated</b> so that positive = forward.
 *
 * <pre>
 * Stick left  = Stick.left(0.05, ((double x) -> x));
 * Stick right = Stick.right();          // defaults
 * // in loop:
 * left.update(gamepad1);
 * double drive  = left.getY();          // deadzone + curve applied, positive = forward
 * double strafe = left.getX();          // deadzone + curve applied, positive = right
 * double mag    = left.getMagnitude();   // combined magnitude [0, 1]
 * double angle  = left.getAngle();      // angle in radians (atan2)
 * </pre>
 */
public class Stick extends GpCommand {

    /** Which physical stick this represents. */
    public enum Side { LEFT, RIGHT }

    public static final Stick LEFT = new Stick(Side.LEFT, 0.05, (double x) -> x);
    public static final Stick RIGHT = new Stick(Side.RIGHT, 0.05, (double x) -> x);

    private final Side side;
    private double deadzone;
    private DoubleUnaryOperator curve;

    // Raw values (after negation for Y)
    private double rawX;
    private double rawY;
    // Processed values
    private double x;
    private double y;
    // speed of the stick
    private double dx;
    private double dy;
    // Button
    private boolean pressed;
    private boolean prevPressed;

    /**
     * Creates a stick reader.
     * @param side     Which stick (LEFT or RIGHT)
     * @param deadzone Values below this magnitude become 0. Range [0, 1). Default 0.05.
     * @param curve    Response-curve exponent (1.0 = linear, 2.0 = squared). ≥ 1.0.
     */
    public Stick(Side side, double deadzone, DoubleUnaryOperator curve) {
        super(TYPE.ANALOG, side == Side.LEFT ? "LEFT_STICK" : "RIGHT_STICK");
        this.side = side;
        this.deadzone = Math.max(0, Math.min(deadzone, 0.99));
        this.curve = curve;
    }

    /** Convenience: creates a LEFT stick with the given deadzone and curve. */
    public static Stick left(double deadzone, DoubleUnaryOperator curve) { return new Stick(Side.LEFT, deadzone, curve); }

    /** Convenience: creates a RIGHT stick with the given deadzone and curve. */
    public static Stick right(double deadzone, DoubleUnaryOperator curve) { return new Stick(Side.RIGHT, deadzone, curve); }

    /** Convenience: creates a LEFT stick with default deadzone (0.05) and linear curve. */
    public static Stick left() { return left(0.05, (double x) -> x); }

    /** Convenience: creates a RIGHT stick with default deadzone (0.05) and linear curve. */
    public static Stick right() { return right(0.05, (double x) -> x); }

    /**
     * Snapshots the stick axes and button from the gamepad.
     * Must be called once per loop iteration.
     * @param gamepad The raw FTC Gamepad
     */
    public void update(Gamepad gamepad) {
        prevPressed = pressed;
        if (side == Side.LEFT) {
            dx = gamepad.left_stick_x - rawX;
            dy = -gamepad.left_stick_y - rawY;   // negate so positive
            rawX = gamepad.left_stick_x;
            rawY = -gamepad.left_stick_y;   // negate so positive = forward
            pressed = gamepad.left_stick_button;
        } else {
            dx = gamepad.right_stick_x - rawX;
            dy = -gamepad.right_stick_y - rawY;   // negate so positive
            rawX = gamepad.right_stick_x;
            rawY = -gamepad.right_stick_y;  // negate so positive = forward
            pressed = gamepad.right_stick_button;
        }
        x = applyStick(rawX);
        y = applyStick(rawY);
    }

    // ==================== PROCESSED QUERIES ====================

    /** @return X with deadzone and curve applied. Positive = right. */
    public double getX() { return x; }

    /** @return Y with deadzone and curve applied. Positive = forward. */
    public double getY() { return y; }

    /** @return Change in X since last update (raw, no deadzone/curve) */
    public double getDeltaX() { return dx; }
    /** @return Change in Y since last update (raw, no deadzone/curve) */
    public double getDeltaY() { return dy; }
    /** @return the change in position of the stick since last update, with deadzone and curve applied */
    public double getDeltaMagnitude() { return Math.hypot(dx, dy); }
    /**
     * @return Combined magnitude of the processed (x, y) vector, clamped to [0, 1].
     *         Useful for speed scaling.
     */
    public double getMagnitude() {
        return Math.min(1.0, Math.hypot(x, y));
    }

    @Override
    public double getValue() {
        return getMagnitude();
    }

    /**
     * @return Angle of the processed (x, y) vector in radians.
     *         0 = right (+X), π/2 = forward (+Y). Returns 0 if magnitude is 0.
     */
    public double getAngle() {
        if (x == 0 && y == 0) return 0;
        return Math.atan2(y, x);
    }

    // ==================== RAW QUERIES ====================

    /** @return Raw X as reported by the gamepad (no deadzone/curve) */
    public double getRawX() { return rawX; }

    /** @return Raw Y <b>negated</b> (positive = forward), no deadzone/curve */
    public double getRawY() { return rawY; }

    // ==================== STICK BUTTON ====================

    /** @return true if the stick button is currently pressed */
    public boolean isButtonPressed() { return pressed; }

    /** @return true on the frame the stick button was first pressed */
    public boolean wasButtonJustPressed() { return pressed && !prevPressed; }

    /** @return true on the frame the stick button was released */
    public boolean wasButtonJustReleased() { return !pressed && prevPressed; }

    // ==================== CONFIGURATION ====================

    /** @return Which physical stick this is */
    public Side getSide() { return side; }

    /**
     * Sets the deadzone.
     * @param deadzone Value in [0, 1). Default 0.05.
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = Math.max(0, Math.min(deadzone, 0.99));
    }

    /** @return The current deadzone */
    public double getDeadzone() { return deadzone; }

    /**
     * Sets the response-curve exponent.
     * @param curve Exponent ≥ 1.0. 1.0 = linear, 2.0 = squared.
     */
    public void setCurve(DoubleUnaryOperator curve) {
        this.curve = curve;
    }

    /** @return The current response-curve exponent */
    public DoubleUnaryOperator getCurve() { return curve; }

    // ==================== INTERNAL ====================

    private double applyStick(double raw) {
        if (Math.abs(raw) < deadzone) return 0.0;
        return curve.applyAsDouble(raw);
    }
}
