package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * Manages edge-detection, toggle state, and queries for <b>every</b> {@link Button}
 * on the gamepad in a single object. Call {@link #update(Gamepad)} once per loop.
 *
 * <pre>
 * Buttons btns = new Buttons();
 * // in loop:
 * btns.update(gamepad1);
 * if (btns.wasJustPressed(Button.A)) { ... }
 * if (btns.getToggle(Button.Y))      { ... }
 * </pre>
 */
public class Buttons {

    private final Map<Button, Reader> readers;

    /** Creates a Buttons tracker with a {@link Reader} for every {@link Button}. */
    public Buttons() {
        readers = new EnumMap<>(Button.class);
        for (Button b : Button.values()) {
            readers.put(b, new Reader(b));
        }
    }

    /**
     * Updates every button reader from the gamepad.
     * Must be called once per loop iteration.
     * @param gamepad The raw FTC Gamepad
     */
    public void update(Gamepad gamepad) {
        for (Reader r : readers.values()) {
            r.update(gamepad);
        }
    }

    /**
     * Gets the {@link Reader} for a specific button for direct access.
     * @param button The button
     * @return The Reader for that button
     */
    public Reader getReader(Button button) {
        return readers.get(button);
    }

    /**
     * @return Unmodifiable map of all readers (Button → Reader)
     */
    public Map<Button, Reader> getReaders() {
        return Collections.unmodifiableMap(readers);
    }

    // ==================== DELEGATED QUERIES ====================

    /** @return true if the button is currently held down */
    public boolean isPressed(Button button) {
        return Objects.requireNonNull(readers.get(button)).isPressed();
    }

    /** @return true on the frame the button was first pressed (rising edge) */
    public boolean wasJustPressed(Button button) {
        return Objects.requireNonNull(readers.get(button)).wasJustPressed();
    }

    /** @return true on the frame the button was released (falling edge) */
    public boolean wasJustReleased(Button button) {
        return Objects.requireNonNull(readers.get(button)).wasJustReleased();
    }

    /**
     * Toggle that flips on every press.
     * @return current toggle state for the button
     */
    public boolean getToggle(Button button) {
        return Objects.requireNonNull(readers.get(button)).getToggle();
    }

    /**
     * Manually overrides the toggle state.
     * @param button The button
     * @param state  desired toggle value
     */
    public void setToggle(Button button, boolean state) {
        Objects.requireNonNull(readers.get(button)).setToggle(state);
    }

    /**
     * @return true if <b>any</b> button is currently held down
     */
    public boolean anyPressed() {
        for (Reader r : readers.values()) {
            if (r.isPressed()) return true;
        }
        return false;
    }

    /**
     * @return true if <b>any</b> button was just pressed this frame
     */
    public boolean anyJustPressed() {
        for (Reader r : readers.values()) {
            if (r.wasJustPressed()) return true;
        }
        return false;
    }
}
