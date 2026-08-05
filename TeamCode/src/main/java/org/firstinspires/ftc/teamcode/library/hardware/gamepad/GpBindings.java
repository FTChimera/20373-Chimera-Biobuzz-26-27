package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * Snapshot-style access to one wrapped controller's already-updated input surfaces.
 *
 * <p>This is a lightweight view over an existing {@link GamepadEx}; it does not
 * call {@link GamepadEx#update()} itself.</p>
 */
public class GpBindings {
    public final Buttons buttons;
    public final Stick leftStick;
    public final Stick rightStick;
    public final Trigger leftTrigger;
    public final Trigger rightTrigger;
    public final Touchpad touchpad;
    private final Map<Button, Reader> readers = new EnumMap<>(Button.class);

    public GpBindings(GamepadEx gamepad) {
        this(gamepad, gamepad.buttons.getReaders());
    }

    public GpBindings(GamepadEx gamepad, Button... buttons) {
        this(gamepad, selectReaders(gamepad.buttons, buttons));
    }

    private GpBindings(GamepadEx gamepad, Map<Button, Reader> readers) {
        Objects.requireNonNull(gamepad, "gamepad");
        this.buttons = gamepad.buttons;
        this.leftStick = gamepad.leftStick;
        this.rightStick = gamepad.rightStick;
        this.leftTrigger = gamepad.leftTrigger;
        this.rightTrigger = gamepad.rightTrigger;
        this.touchpad = gamepad.touchpad;
        this.readers.putAll(Objects.requireNonNull(readers, "readers"));
    }

    public Reader getReader(Button button) {
        return readers.get(button);
    }

    public boolean contains(Button button) {
        return readers.containsKey(button);
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

    private static Map<Button, Reader> selectReaders(Buttons buttons, Button... selectedButtons) {
        Objects.requireNonNull(buttons, "buttons");
        Map<Button, Reader> selected = new EnumMap<>(Button.class);
        if (selectedButtons == null || selectedButtons.length == 0) {
            selected.putAll(buttons.getReaders());
            return selected;
        }
        for (Button button : selectedButtons) {
            selected.put(button, Objects.requireNonNull(buttons.getReader(button), "reader"));
        }
        return selected;
    }
}
