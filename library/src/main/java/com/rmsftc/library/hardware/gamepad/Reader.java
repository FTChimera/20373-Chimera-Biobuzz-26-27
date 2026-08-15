package com.rmsftc.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Tracks the state of a single {@link Button} across frames and provides
 * edge-detection (rising / falling), hold state, and a toggle.
 *
 * <p>Create one per button you want to track, then call {@link #update(Gamepad)}
 * once per loop <b>before</b> reading state.
 *
 * <pre>
 * Reader aReader = new Reader(Button.A);
 * // in loop:
 * aReader.update(gamepad1);
 * if (aReader.wasJustPressed()) { ... }
 * </pre>
 */
public class Reader {

    private final Button button;
    private boolean current;
    private boolean previous;
    private boolean toggleState;

    /**
     * Creates a reader for the given button.
     * @param button The button to track
     */
    public Reader(Button button) {
        this.button = button;
    }

    /**
     * Snapshots the button state from the gamepad.
     * Must be called once per loop iteration.
     * @param gamepad The raw FTC Gamepad
     */
    public void update(Gamepad gamepad) {
        previous = current;
        current = button.isPressed(gamepad);
        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    /** @return The {@link Button} this reader tracks */
    public Button getButton() { return button; }

    /** @return true if the button is currently held down */
    public boolean isPressed() { return current; }

    /** @return true on the frame the button was first pressed (rising edge) */
    public boolean wasJustPressed() { return current && !previous; }

    /** @return true on the frame the button was released (falling edge) */
    public boolean wasJustReleased() { return !current && previous; }

    /**
     * Toggle that flips on every press.
     * Starts {@code false}; first press → {@code true}; second press → {@code false}, etc.
     * @return current toggle state
     */
    public boolean getToggle() { return toggleState; }

    /**
     * Manually overrides the toggle state.
     * @param state desired toggle value
     */
    public void setToggle(boolean state) { this.toggleState = state; }
}