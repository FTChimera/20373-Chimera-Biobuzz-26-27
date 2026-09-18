package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

/**
 * Descriptor and live state holder for one discrete FTC gamepad button, including
 * PS4/PS5 touchpad click.
 * Each entry carries a {@link Function} that extracts the current boolean state from
 * a raw {@link Gamepad}.
 *
 * <pre>
 * boolean pressed = Button.A.isPressed(gamepad1);
 * </pre>
 */
public class Button extends GpCommand {
    private static final List<Button> VALUES = new ArrayList<>();

    public static final Button A = register("A", g -> g.a);
    public static final Button B = register("B", g -> g.b);
    public static final Button X = register("X", g -> g.x);
    public static final Button Y = register("Y", g -> g.y);
    public static final Button DPAD_UP = register("DPAD_UP", g -> g.dpad_up);
    public static final Button DPAD_DOWN = register("DPAD_DOWN", g -> g.dpad_down);
    public static final Button DPAD_LEFT = register("DPAD_LEFT", g -> g.dpad_left);
    public static final Button DPAD_RIGHT = register("DPAD_RIGHT", g -> g.dpad_right);
    public static final Button LEFT_BUMPER = register("LEFT_BUMPER", g -> g.left_bumper);
    public static final Button RIGHT_BUMPER = register("RIGHT_BUMPER", g -> g.right_bumper);
    public static final Button LEFT_STICK_BUTTON = register("LEFT_STICK_BUTTON", g -> g.left_stick_button);
    public static final Button RIGHT_STICK_BUTTON = register("RIGHT_STICK_BUTTON", g -> g.right_stick_button);
    public static final Button BACK = register("BACK", g -> g.back);
    public static final Button START = register("START", g -> g.start);
    public static final Button OPTIONS = register("OPTIONS", g -> g.options);
    public static final Button GUIDE = register("GUIDE", g -> g.guide);
    public static final Button TOUCHPAD = register("TOUCHPAD", g -> g.touchpad);

    private final Function<Gamepad, Boolean> reader;
    private final int ordinal;
    private boolean current;
    private boolean previous;
    private boolean toggleState;

    private Button(String name, Function<Gamepad, Boolean> reader, int ordinal) {
        super(TYPE.BUTTON, name);
        this.reader = reader;
        this.ordinal = ordinal;
    }

    private Button(Button other) {
        this(other.getName(), other.reader, other.ordinal);
    }

    private static Button register(String name, Function<Gamepad, Boolean> reader) {
        Button button = new Button(name, reader, VALUES.size());
        VALUES.add(button);
        return button;
    }

    public static Button[] values() {
        return VALUES.toArray(new Button[0]);
    }

    public static List<Button> valuesList() {
        return Collections.unmodifiableList(VALUES);
    }

    public Button copy() {
        return new Button(this);
    }

    int ordinal() {
        return ordinal;
    }

    public void update(Gamepad gamepad) {
        previous = current;
        current = isPressed(gamepad);
        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    /**
     * Reads the current state of this button from the raw gamepad.
     * @param gamepad The FTC Gamepad
     * @return true if the button is currently pressed
     */
    public boolean isPressed(Gamepad gamepad) {
        return reader.apply(gamepad);
    }

    public boolean isPressed() {
        return current;
    }

    @Override
    public double getValue() {
        return current ? 1.0 : 0.0;
    }

    public boolean wasJustPressed() {
        return current && !previous;
    }

    public boolean wasJustReleased() {
        return !current && previous;
    }

    public boolean getToggle() {
        return toggleState;
    }

    public void setToggle(boolean state) {
        toggleState = state;
    }
}
