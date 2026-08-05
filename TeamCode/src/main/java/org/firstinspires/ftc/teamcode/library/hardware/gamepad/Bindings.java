package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * A lightweight list of bound buttons and their already-updated readers.
 *
 * <p>This does not own a second input-update loop. Instead, it references
 * readers from {@link Buttons} / {@link GamepadEx}, so callers should keep
 * updating the original wrapper once per loop before reading binding state.</p>
 */
public class Bindings {
    private final Map<Button, Reader> readers = new EnumMap<>(Button.class);

    public Bindings bind(Button button, Reader reader) {
        readers.put(Objects.requireNonNull(button, "button"), Objects.requireNonNull(reader, "reader"));
        return this;
    }

    public boolean contains(Button button) {
        return readers.containsKey(button);
    }

    public Reader getReader(Button button) {
        return readers.get(button);
    }

    public Map<Button, Reader> getReaders() {
        return Collections.unmodifiableMap(readers);
    }

    public boolean isPressed(Button button) {
        Reader reader = readers.get(button);
        return reader != null && reader.isPressed();
    }

    public boolean wasJustPressed(Button button) {
        Reader reader = readers.get(button);
        return reader != null && reader.wasJustPressed();
    }

    public boolean wasJustReleased(Button button) {
        Reader reader = readers.get(button);
        return reader != null && reader.wasJustReleased();
    }

    public boolean getToggle(Button button) {
        Reader reader = readers.get(button);
        return reader != null && reader.getToggle();
    }

    public static Bindings empty() {
        return new Bindings();
    }

    public static Bindings of(Buttons buttons, Button... boundButtons) {
        Objects.requireNonNull(buttons, "buttons");
        Bindings bindings = new Bindings();
        for (Button button : boundButtons) {
            bindings.bind(button, Objects.requireNonNull(buttons.getReader(button), "reader"));
        }
        return bindings;
    }

    public static Bindings of(GamepadEx gamepad, Button... boundButtons) {
        Objects.requireNonNull(gamepad, "gamepad");
        return of(gamepad.buttons, boundButtons);
    }
}
