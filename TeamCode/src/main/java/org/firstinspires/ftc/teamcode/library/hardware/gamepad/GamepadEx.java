package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

/**
 * Extended gamepad wrapper that composes {@link Buttons}, {@link Stick}, {@link Trigger},
 * and {@link Touchpad} components together with PS4/PS5 LED light-bar and rumble control.
 *
 * <p>Call {@link #update()} once per loop <b>before</b> reading any values.
 *
 * <pre>
 * GamepadEx gp1 = new GamepadEx(gamepad1);
 * // in loop:
 * gp1.update();
 *
 * // Buttons
 * if (gp1.buttons.wasJustPressed(Button.A)) { ... }
 * if (gp1.buttons.getToggle(Button.Y))      { ... }
 *
 * // Sticks (deadzone + curve applied)
 * double drive  = gp1.leftStick.getY();
 * double strafe = gp1.leftStick.getX();
 * double turn   = gp1.rightStick.getX();
 *
 * // Triggers
 * if (gp1.leftTrigger.wasJustPressed()) { ... }
 * double rt = gp1.rightTrigger.getValue();
 *
 * // Touchpad (PS4/PS5)
 * for (Touchpad.TouchPoint tp : gp1.touchpad.getFingers()) {
 *     telemetry.addData("Finger", "x=%.2f y=%.2f", tp.x, tp.y);
 * }
 *
 * // LED light bar (PS4/PS5)
 * gp1.setLedColor(0, 1, 0, 500);   // green for 500ms
 * gp1.setLedColor(1, 0, 0);        // red continuous
 *
 * // Rumble
 * gp1.rumble(500);
 * gp1.runRumbleEffect(myEffect);
 * </pre>
 */

@SuppressWarnings("SpellCheckingInspection")
public class GamepadEx {

    // ==================== COMPONENTS (public for direct access) ====================

    /** The raw FTC Gamepad this wrapper reads from. */
    public final Gamepad gamepad;

    /** Detected controller type and its capabilities. */
    public final GamepadType type;

    /** All discrete buttons with edge-detection and toggles. */
    public final Buttons buttons;

    /** Left analog stick with deadzone, curve, magnitude, angle, and stick-button. */
    public final Stick leftStick;

    /** Right analog stick with deadzone, curve, magnitude, angle, and stick-button. */
    public final Stick rightStick;

    /** Left analog trigger with threshold, edge-detection, and toggle. */
    public final Trigger leftTrigger;

    /** Right analog trigger with threshold, edge-detection, and toggle. */
    public final Trigger rightTrigger;

    /** PS4/PS5 touchpad with multi-finger tracking and button click. */
    public final Touchpad touchpad;

    // ==================== CONSTRUCTORS ====================

    /**
     * Creates a GamepadEx with default settings (deadzone 0.05, linear curve,
     * trigger threshold 0.5).
     * @param gamepad The raw FTC Gamepad (e.g. {@code gamepad1})
     */
    public GamepadEx(Gamepad gamepad) {
        this(gamepad, 0.05, (DoubleUnaryOperator) ((double x) -> x), 0.5);
    }

    /**
     * Creates a GamepadEx with custom deadzone, curve, and trigger threshold.
     * @param gamepad   The raw FTC Gamepad
     * @param deadzone  Stick deadzone [0, 1). Default 0.05.
     * @param curve     Stick response curve exponent (≥ 1.0). 1.0 = linear.
     * @param triggerThreshold Trigger digital threshold (0, 1]. Default 0.5.
     */
    public GamepadEx(Gamepad gamepad, double deadzone, DoubleUnaryOperator curve, double triggerThreshold) {
        this.gamepad = gamepad;
        this.type = GamepadType.fromGamepad(gamepad);
        this.buttons = new Buttons();
        this.leftStick = Stick.left(deadzone, curve);
        this.rightStick = Stick.right(deadzone, curve);
        this.leftTrigger = Trigger.left(triggerThreshold);
        this.rightTrigger = Trigger.right(triggerThreshold);
        this.touchpad = new Touchpad();
    }

    // ==================== UPDATE ====================

    /**
     * Must be called <b>once per loop iteration</b>, at the start of the loop,
     * before reading any values. Updates all components from the raw gamepad.
     */
    public void update() {
        buttons.update(gamepad);
        leftStick.update(gamepad);
        rightStick.update(gamepad);
        leftTrigger.update(gamepad);
        rightTrigger.update(gamepad);
        touchpad.update(gamepad);
    }

    // ==================== CONVENIENCE: BUTTONS ====================

    /** @see Buttons#isPressed(Button) */
    public boolean isPressed(Button button) { return buttons.isPressed(button); }

    /** @see Buttons#wasJustPressed(Button) */
    public boolean wasJustPressed(Button button) { return buttons.wasJustPressed(button); }

    /** @see Buttons#wasJustReleased(Button) */
    public boolean wasJustReleased(Button button) { return buttons.wasJustReleased(button); }

    /** @see Buttons#getToggle(Button) */
    public boolean getToggle(Button button) { return buttons.getToggle(button); }

    /** @see Buttons#setToggle(Button, boolean) */
    public void setToggle(Button button, boolean state) { buttons.setToggle(button, state); }

    /** Creates a bindings view over selected button readers from this wrapper. */
    public Bindings bindings(Button... buttons) { return Bindings.of(this, buttons); }

    // ==================== CONVENIENCE: STICKS ====================

    /** @return Left stick X with deadzone and curve applied. Positive = right. */
    public double leftStickX() { return leftStick.getX(); }

    /** @return Left stick Y with deadzone and curve applied. Positive = forward. */
    public double leftStickY() { return leftStick.getY(); }

    /** @return Right stick X with deadzone and curve applied. Positive = right. */
    public double rightStickX() { return rightStick.getX(); }

    /** @return Right stick Y with deadzone and curve applied. Positive = forward. */
    public double rightStickY() { return rightStick.getY(); }

    // ==================== CONVENIENCE: TOUCHPAD ====================

    /**
     * Returns a list of {@link Touchpad.TouchPoint}s for all fingers currently on
     * the PS4/PS5 touchpad. The list contains 0, 1, or 2 elements.
     * @return Unmodifiable list of active touch points
     */
    public List<Touchpad.TouchPoint> getTouchpadFingers() { return touchpad.getFingers(); }

    // ==================== CONFIGURATION ====================

    /**
     * Sets the stick deadzone for both sticks.
     * @param deadzone Value in [0, 1). Default 0.05.
     * @return this for chaining
     */
    public GamepadEx setJoystickDeadzone(double deadzone) {
        leftStick.setDeadzone(deadzone);
        rightStick.setDeadzone(deadzone);
        return this;
    }

    /**
     * Sets the stick response curve exponent for both sticks.
     * @param curve the function to apply to stick inputs
     * @return this for chaining
     */
    public GamepadEx setStickCurve(DoubleUnaryOperator curve) {
        leftStick.setCurve(curve);
        rightStick.setCurve(curve);
        return this;
    }

    /**
     * Sets the trigger digital threshold for both triggers.
     * @param threshold Value in (0, 1]. Default 0.5.
     * @return this for chaining
     */
    public GamepadEx setTriggerThreshold(double threshold) {
        leftTrigger.setThreshold(threshold);
        rightTrigger.setThreshold(threshold);
        return this;
    }

    // ==================== LED LIGHT BAR (PS4/PS5) ====================

    /**
     * Sets the PS4/PS5 controller light-bar LED color for a specified duration.
     * Has no effect on gamepads without an LED light bar.
     *
     * @param red   Red channel [0.0, 1.0]
     * @param green Green channel [0.0, 1.0]
     * @param blue  Blue channel [0.0, 1.0]
     * @param durationMs Duration in milliseconds, or {@link Gamepad#LED_DURATION_CONTINUOUS}
     */
    public void setLedColor(double red, double green, double blue, int durationMs) {
        gamepad.setLedColor(red, green, blue, durationMs);
    }

    /**
     * Sets the PS4/PS5 controller light-bar LED color continuously (until changed).
     *
     * @param red   Red channel [0.0, 1.0]
     * @param green Green channel [0.0, 1.0]
     * @param blue  Blue channel [0.0, 1.0]
     */
    public void setLedColor(double red, double green, double blue) {
        gamepad.setLedColor(red, green, blue, Gamepad.LED_DURATION_CONTINUOUS);
    }

    /** Turns off the PS4/PS5 controller light-bar LED. */
    public void ledOff() {
        gamepad.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    // ==================== RUMBLE ====================

    /**
     * Rumbles both motors for the given duration.
     * @param durationMs Duration in milliseconds
     */
    public void rumble(int durationMs) { gamepad.rumble(durationMs); }

    /**
     * Rumbles with independent left/right motor control.
     * @param left   Left motor power [0, 1]
     * @param right  Right motor power [0, 1]
     * @param durationMs Duration in milliseconds, or {@link Gamepad#RUMBLE_DURATION_CONTINUOUS}
     */
    public void rumble(double left, double right, int durationMs) {
        gamepad.rumble(left, right, durationMs);
    }

    /**
     * Rumbles with independent left/right motor control continuously.
     * @param left  Left motor power [0, 1]
     * @param right Right motor power [0, 1]
     */
    public void rumbleContinuous(double left, double right) {
        gamepad.rumble(left, right, Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    /**
     * Sends a series of short rumble blips.
     * @param count Number of blips
     */
    public void rumbleBlips(int count) { gamepad.rumbleBlips(count); }

    /**
     * Runs a custom rumble effect built with {@link Gamepad.RumbleEffect.Builder}.
     * @param effect The custom rumble effect to play
     */
    public void runRumbleEffect(Gamepad.RumbleEffect effect) {
        gamepad.runRumbleEffect(effect);
    }

    /** Stops any in-progress rumble or custom rumble effect. */
    public void stopRumble() { gamepad.stopRumble(); }

    /** @return true if the gamepad is currently rumbling */
    public boolean isRumbling() { return gamepad.isRumbling(); }

    // ==================== UTILITIES ====================

    /**
     * @return true if any button, trigger, stick, or touchpad finger is currently active.
     *         Useful for "press anything to continue" prompts.
     */
    public boolean anyInput() {
        return buttons.anyPressed()
                || leftTrigger.isPressed()
                || rightTrigger.isPressed()
                || touchpad.isTouched()
                || leftStick.getMagnitude() > 0
                || rightStick.getMagnitude() > 0;
    }

    /**
     * @return The underlying raw FTC {@link Gamepad} object
     */
    public Gamepad getRawGamepad() { return gamepad; }

    /**
     * @return The detected {@link GamepadType}
     */
    public GamepadType getType() { return type; }
}
