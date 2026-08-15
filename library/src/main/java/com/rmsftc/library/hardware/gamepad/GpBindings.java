package com.rmsftc.library.hardware.gamepad;

import java.util.Collections;
import java.util.LinkedHashMap;
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
    private final Map<Button, Button> readers = new LinkedHashMap<>();

    public GpBindings(GamepadEx gamepad) {
        this(gamepad, (GpCommand[]) null);
    }

    public GpBindings(GamepadEx gamepad, GpCommand... commands) {
        Objects.requireNonNull(gamepad, "gamepad must not be null");
        this.buttons = gamepad.buttons;
        this.leftStick = gamepad.leftStick;
        this.rightStick = gamepad.rightStick;
        this.leftTrigger = gamepad.leftTrigger;
        this.rightTrigger = gamepad.rightTrigger;
        this.touchpad = gamepad.touchpad;
        if (commands == null || commands.length == 0) {
            this.readers.putAll(Objects.requireNonNull(gamepad.buttons.getReaders(), "readers must not be null"));
        } else {
            for (GpCommand command : commands) {
                if (command instanceof Button) {
                    Button button = gamepad.buttons.getReader((Button) command);
                    if (button != null) {
                        readers.put((Button) command, button);
                    }
                }
            }
        }
    }

    public Button getReader(Button button) {
        return readers.get(button);
    }

    public boolean contains(Button button) {
        return readers.containsKey(button);
    }

    public Map<Button, Button> getReaders() {
        return Collections.unmodifiableMap(readers);
    }

    public boolean isPressed(Button button) {
        Button reader = readers.get(button);
        return reader != null && reader.isPressed();
    }

    public boolean wasJustPressed(Button button) {
        Button reader = readers.get(button);
        return reader != null && reader.wasJustPressed();
    }

    public boolean wasJustReleased(Button button) {
        Button reader = readers.get(button);
        return reader != null && reader.wasJustReleased();
    }

    public boolean getToggle(Button button) {
        Button reader = readers.get(button);
        return reader != null && reader.getToggle();
    }

    @SuppressWarnings("unchecked")
    public <T extends GpCommand> T getCommand(T command) {
        if (command == null) {
            return null;
        }
        if (command instanceof Button) {
            return (T) readers.get(command);
        } else if (command instanceof Stick) {
            Stick stick = (Stick) command;
            return (T) (stick.getSide() == Stick.Side.LEFT ? leftStick : rightStick);
        } else if (command instanceof Trigger) {
            Trigger trigger = (Trigger) command;
            return (T) (trigger.getSide() == Trigger.Side.LEFT ? leftTrigger : rightTrigger);
        } else if (command instanceof Touchpad) {
            return (T) touchpad;
        } else {
            return null;
        }
    }

}
