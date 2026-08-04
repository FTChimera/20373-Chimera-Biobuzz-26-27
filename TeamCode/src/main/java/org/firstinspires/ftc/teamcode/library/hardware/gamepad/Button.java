package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

/**
 * Enum of all discrete buttons on an FTC gamepad, including PS4/PS5 touchpad click.
 * Each entry carries a {@link Function} that extracts the current boolean state from
 * a raw {@link Gamepad}.
 *
 * <pre>
 * boolean pressed = Button.A.isPressed(gamepad1);
 * </pre>
 */
public enum Button {
    A           (g -> g.a),
    B           (g -> g.b),
    X           (g -> g.x),
    Y           (g -> g.y),
    DPAD_UP     (g -> g.dpad_up),
    DPAD_DOWN   (g -> g.dpad_down),
    DPAD_LEFT   (g -> g.dpad_left),
    DPAD_RIGHT  (g -> g.dpad_right),
    LEFT_BUMPER (g -> g.left_bumper),
    RIGHT_BUMPER(g -> g.right_bumper),
    LEFT_STICK_BUTTON (g -> g.left_stick_button),
    RIGHT_STICK_BUTTON(g -> g.right_stick_button),
    BACK        (g -> g.back),
    START       (g -> g.start),
    OPTIONS     (g -> g.options),
    GUIDE       (g -> g.guide),
    /** PS4/PS5 touchpad click (pressing the touchpad down as a button). */
    TOUCHPAD    (g -> g.touchpad);

    private final Function<Gamepad, Boolean> reader;

    Button(Function<Gamepad, Boolean> reader) {
        this.reader = reader;
    }

    /**
     * Reads the current state of this button from the raw gamepad.
     * @param gamepad The FTC Gamepad
     * @return true if the button is currently pressed
     */
    public boolean isPressed(Gamepad gamepad) {
        return reader.apply(gamepad);
    }
}
