package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Snapshot-style access to one or two wrapped controllers' already-updated input surfaces.
 *
 * <p>This is a lightweight view over an existing {@link GamepadEx}; it does not
 * call {@link GamepadEx#update()} itself.</p>
 */
public class GpBindings {
    public enum GamepadId {
        ONE,
        TWO
    }

    public final Buttons buttons;
    public final Stick leftStick;
    public final Stick rightStick;
    public final Trigger leftTrigger;
    public final Trigger rightTrigger;
    public final Touchpad touchpad;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final Map<Button, Button> readers1 = new LinkedHashMap<>();
    private final Map<Button, Button> readers2 = new LinkedHashMap<>();

    public GpBindings(GamepadEx gamepad) {
        this(gamepad, (GpCommand[]) null);
    }

    public GpBindings(GamepadEx gamepad, GpCommand... commands) {
        this(gamepad, null, commands);
    }

    public GpBindings(DualGamepad gamepads) {
        this(gamepads, (GpCommand[]) null);
    }

    public GpBindings(DualGamepad gamepads, GpCommand... commands) {
        this(
                Objects.requireNonNull(gamepads, "gamepads must not be null").gamepad1,
                gamepads.gamepad2,
                commands
        );
    }

    private GpBindings(GamepadEx gamepad1, GamepadEx gamepad2, GpCommand... commands) {
        this.gamepad1 = Objects.requireNonNull(gamepad1, "gamepad1 must not be null");
        this.gamepad2 = gamepad2;
        this.buttons = gamepad1.buttons;
        this.leftStick = gamepad1.leftStick;
        this.rightStick = gamepad1.rightStick;
        this.leftTrigger = gamepad1.leftTrigger;
        this.rightTrigger = gamepad1.rightTrigger;
        this.touchpad = gamepad1.touchpad;
        addReaders(gamepad1, readers1, commands);
        if (gamepad2 != null) {
            addReaders(gamepad2, readers2, commands);
        }
    }

    private static void addReaders(GamepadEx gamepad, Map<Button, Button> readers, GpCommand... commands) {
        if (commands == null || commands.length == 0) {
            readers.putAll(Objects.requireNonNull(gamepad.buttons.getReaders(), "readers must not be null"));
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
        return getReader(GamepadId.ONE, button);
    }

    public Button getReader(GamepadId gamepad, Button button) {
        return getReadersFor(gamepad).get(button);
    }

    public boolean contains(Button button) {
        return contains(GamepadId.ONE, button);
    }

    public boolean contains(GamepadId gamepad, Button button) {
        return getReadersFor(gamepad).containsKey(button);
    }

    public Map<Button, Button> getReaders() {
        return getReaders(GamepadId.ONE);
    }

    public Map<Button, Button> getReaders(GamepadId gamepad) {
        return Collections.unmodifiableMap(getReadersFor(gamepad));
    }

    public boolean isPressed(Button button) {
        return isPressed(GamepadId.ONE, button);
    }

    public boolean isPressed(GamepadId gamepad, Button button) {
        Button reader = getReader(gamepad, button);
        return reader != null && reader.isPressed();
    }

    public boolean wasJustPressed(Button button) {
        return wasJustPressed(GamepadId.ONE, button);
    }

    public boolean wasJustPressed(GamepadId gamepad, Button button) {
        Button reader = getReader(gamepad, button);
        return reader != null && reader.wasJustPressed();
    }

    public boolean wasJustReleased(Button button) {
        return wasJustReleased(GamepadId.ONE, button);
    }

    public boolean wasJustReleased(GamepadId gamepad, Button button) {
        Button reader = getReader(gamepad, button);
        return reader != null && reader.wasJustReleased();
    }

    public boolean getToggle(Button button) {
        return getToggle(GamepadId.ONE, button);
    }

    public boolean getToggle(GamepadId gamepad, Button button) {
        Button reader = getReader(gamepad, button);
        return reader != null && reader.getToggle();
    }

    public <T extends GpCommand> T getCommand(T command) {
        return getCommand(GamepadId.ONE, command);
    }

    @SuppressWarnings("unchecked")
    public <T extends GpCommand> T getCommand(GamepadId gamepad, T command) {
        if (command == null) {
            return null;
        }
        GamepadEx selectedGamepad = getGamepad(gamepad);
        if (selectedGamepad == null) {
            return null;
        }
        if (command instanceof Button) {
            return (T) getReader(gamepad, (Button) command);
        } else if (command instanceof Stick) {
            Stick stick = (Stick) command;
            return (T) (stick.getSide() == Stick.Side.LEFT ? selectedGamepad.leftStick : selectedGamepad.rightStick);
        } else if (command instanceof Trigger) {
            Trigger trigger = (Trigger) command;
            return (T) (trigger.getSide() == Trigger.Side.LEFT ? selectedGamepad.leftTrigger : selectedGamepad.rightTrigger);
        } else if (command instanceof Touchpad) {
            return (T) selectedGamepad.touchpad;
        } else {
            return null;
        }
    }

    private GamepadEx getGamepad(GamepadId gamepad) {
        Objects.requireNonNull(gamepad, "gamepad must not be null");
        return gamepad == GamepadId.ONE ? gamepad1 : gamepad2;
    }

    private Map<Button, Button> getReadersFor(GamepadId gamepad) {
        Objects.requireNonNull(gamepad, "gamepad must not be null");
        return gamepad == GamepadId.ONE ? readers1 : readers2;
    }
}
