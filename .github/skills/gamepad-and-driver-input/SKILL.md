---
name: gamepad-and-driver-input
description: Use this skill for FTC gamepad controls and project input wrappers.
---

# Gamepad and Driver Input

Use the wrapper types so edge detection and two-controller state stay consistent.

## Main types

- `DualGamepad` wraps SDK `gamepad1` and `gamepad2`.
- It exposes `GamepadEx` instances as `gamepad1` and `gamepad2`.
- `GamepadEx` owns buttons, sticks, triggers, and touchpad readers.
- `GpBindings` is a read-only view over already-updated controller state.
- `State` carries the active bindings to mechanisms.

## Standard teleop flow

1. Construct `DualGamepad(gamepad1, gamepad2)` during initialization.
2. Create a `GpBindings` view with `drivers.result()`.
3. Put that view on the teleop `State`.
4. Call `drivers.update()` once per active loop.
5. Call `robot.periodic(state)` after the update.
6. Consume current and edge-triggered values from bindings or wrappers.

## Update rule

- `DualGamepad.update()` updates both wrapped controllers.
- It must run before reading pressed, released, or toggle state for that loop.
- `GpBindings` does not call update itself.
- Do not create a new wrapper every loop.
- Do not update one controller while reading stale values from the other.
- `updateCommand()` returns an Ivy infinite command when scheduler ownership is appropriate.

## Controller access

- `getGamepad1()` and `getGamepad2()` return wrapped controllers.
- `getRawGamepad1()` and `getRawGamepad2()` return FTC SDK gamepads.
- Prefer wrapper sticks, triggers, and buttons for normal robot logic.
- Use raw SDK access only when a wrapper lacks the needed surface.
- Preserve controller identity rather than encoding it into button names.

## Bindings

- `result()` exposes bindings for both controllers.
- `result(GpCommand...)` can expose selected commands eagerly.
- `GpBindings.GamepadId.ONE` and `TWO` choose controller-specific access.
- The overload without an ID reads controller one for compatibility.
- `getCommand(id, command)` resolves buttons, sticks, triggers, and touchpad objects.
- A missing controller-two command returns `null`; handle it intentionally.

## Buttons and toggles

- Use `isPressed` for continuous action.
- Use `wasJustPressed` and `wasJustReleased` for edges.
- Use toggles only when their reset behavior is specified.
- `DualGamepad` provides controller-specific button and toggle helpers.
- Avoid combining raw button state and wrapper edge state for the same action.

## Analog controls

- Use the wrapped left and right stick objects for each controller.
- Use wrapped triggers to apply configured thresholds.
- Set a shared deadzone, curve, or trigger threshold through `DualGamepad` when appropriate.
- Avoid repeatedly changing input configuration inside the match loop.
- Drive stick shaping belongs with the configured drive path, not with unrelated mechanisms.

## Ivy usage

- Schedule `DualGamepad.updateCommand()` only when the scheduler is actively executed.
- Ensure exactly one update owner exists for a given loop.
- Command-based input does not replace setting bindings on the robot `State`.
- Keep command cancellation and robot stop behavior coordinated.

## Validation

- Use existing gamepad unit tests for edge, toggle, and binding behavior.
- Test both physical controllers on the Driver Station.
- Test held, tapped, and release actions.
- Test controller-two bindings explicitly.
- Verify an input action remains safe when an optional mechanism is absent.

## Control design

- Assign each action a clear controller and button surface.
- Prefer edge actions for toggles, presets, and scheduler requests.
- Prefer continuous values for drive, intake power, and manual mechanisms.
- Make simultaneous-controller behavior explicit instead of relying on ordering.
- Preserve a safe default when a button is not pressed.
- Surface controller state in telemetry while debugging bindings.
- Reset or initialize toggle state intentionally at match start.
- Review actions after changing the active robot profile.
- Keep driver ergonomics separate from low-level hardware calibration.

## Change boundaries

- Keep raw FTC gamepad interaction at the wrapper boundary.
- Keep action interpretation in mechanisms or teleop orchestration.
- Keep motor and sensor behavior out of general binding classes.
- Re-test controller ownership after moving an action between these layers.
