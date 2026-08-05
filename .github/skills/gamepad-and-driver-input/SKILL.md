---
name: gamepad-and-driver-input
description: Use this skill when asked to work on driver controls, raw FTC gamepad calls, or the repo’s `GamepadEx` / button-reader abstractions.
---

# Gamepad and Driver Input

This skill covers the TeamCode gamepad wrapper utilities in `library/hardware/gamepad` and how they replace direct FTC gamepad reads in repo code.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/DualGamepad.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/GamepadEx.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/GpBindings.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Button.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Buttons.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Reader.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Stick.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Touchpad.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Trigger.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MotorDirectionTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`

## Current repo usage

- TeamCode op modes and tests now prefer `GamepadEx` or `DualGamepad` instead of direct `gamepad1` / `gamepad2` field reads.
- `TeleOp`, `LLTester`, `PreseasonWebcamAuto`, `MotorDirectionTest`, `PedroPathingTeleOpTest`, `FieldCentricMecanumTeleOp`, and the FTC SDK test wrappers now all use the wrapper surface.
- `IvyTeleOp` is the main `DualGamepad.updateCommand()` example.
- `GpBindings` is the wrapper-side aggregate of one gamepad’s current selected button readers, sticks, triggers, and touchpad state.
- `MecanumTeleOp` still applies its own deadzone and response curve inline, but it now reads buttons, triggers, and raw stick values through a wrapper configured with neutral stick shaping.
- `MecanumTeleOp` now also stays aligned with the active drivetrain profile by sourcing motor names and default directions from `Constants.currentBot`.
- `GamepadEx` exists as a richer wrapper with buttons, sticks, triggers, touchpad, LED, and rumble support.
- `GamepadEx.result()` and `GamepadEx.result(Button...)` create a `GpBindings` view that higher-level helpers can pass around without reaching back into raw FTC gamepad state.
- `DualGamepad` wraps two `GamepadEx` objects and is the repo’s dual-controller utility surface when one abstraction needs to manage both `gamepad1` and `gamepad2` together.
- `Button.OPTIONS` now exists because TeamCode uses the wrapper to cover the FTC `options` button path as well as the face buttons, dpad, and back/start controls.
- `OpModeHelper.kt` is available for loop timing and manual bulk-caching work, but it is separate from the input wrappers and should not become a second button/joystick abstraction.

## `DualGamepad` conventions

When a task needs one object that manages both drivers/controllers, follow the existing `DualGamepad` pattern instead of inventing a new combined wrapper.

### Shape of the wrapper

- `DualGamepad` owns two wrapped `GamepadEx` objects: `gamepad1` and `gamepad2`.
- Keep one combined `update()` method that updates both wrapped controllers in the same call.
- Keep one Ivy helper `updateCommand()` that continuously updates both controllers together.
- Expose per-pad accessors for values that are inherently different across controllers.

### Update rules

- Call `dualGamepad.update()` once per loop before reading button, stick, trigger, or touchpad state.
- If using Ivy, `dualGamepad.updateCommand()` is the preferred continuous update hook.
- Do not create separate loop-level `update1()` / `update2()` methods when both controllers should stay in sync.

### Naming rules

- Combined behavior that should affect both pads may use one shared method, such as `update()`, `setStickCurve(...)`, `setTriggerThreshold(...)`, `rumble(...)`, or `ledOff()`.
- Per-controller state should use explicit per-pad naming such as `getTouchpad1()`, `getTouchpad2()`, `getTouchpadFingers1()`, and `getTouchpadFingers2()`.
- Prefer clear names like `getTouchpad1Finger1()` and `getTouchpad2Finger2()` over awkward numeric mashups like `getTouchpadFinger21()`.
- If compatibility aliases exist for old names, keep them deprecated and steer new code toward the clearer names.

### Stick and activity expectations

- Stick accessors on `DualGamepad` should return the actual `Stick` objects, not just raw doubles.
- The canonical stick-style methods are `leftStick1()`, `leftStick2()`, `rightStick1()`, and `rightStick2()`.
- Keep explicit activity helpers: `isGamepad1Active()`, `isGamepad2Active()`, and `isEitherGamepadActive()`.
- `anyInput1()`, `anyInput2()`, and `anyInput()` may exist as aliases, but the more explicit `isGamepad...Active()` names are preferred in new code.

### Per-pad touchpad expectations

Touchpad and finger-specific information should stay separate per controller, for example:

- `getTouchpad1()` / `getTouchpad2()`
- `getTouchpadFingers1()` / `getTouchpadFingers2()`
- `getTouchpad1Finger1()` / `getTouchpad2Finger1()`
- `isTouchpad1Touched()` / `isTouchpad2Touched()`
- `wasTouchpad1ButtonJustPressed()` / `wasTouchpad2ButtonJustPressed()`

Do not collapse finger-specific or touchpad-specific queries into a single ambiguous combined method.

## Rules

- Prefer the wrapper surface over raw FTC gamepad reads in TeamCode op modes and tests.
- If you need reusable input behavior like toggles, edge detection, touchpad tracking, or controller LED support, extend the wrapper utilities instead of recreating them ad hoc.
- Use `GpBindings` when a subsystem or periodic path should depend on selected button readers plus analog inputs from the same wrapper object.
- Call wrapper `update()` methods once per loop before reading values.
- Keep stick-curve logic consistent with drive expectations; the repo often applies shaping in `MecanumDrive` / `DriveVector`, not directly in every button handler.
- When a new button action needs a new mechanism, add that mechanism through the normal `Mechanism` / `RobotConstants.createMechanism(...)` flow before wiring buttons to it.
- For a dual-controller wrapper, keep one combined update flow and separate per-pad accessors instead of mixing both styles inconsistently.
- If you introduce wrappers into an op mode that was previously raw-FTC-only, document whether the wrapper is replacing or just augmenting the old style.

## Good use cases for wrappers

- Toggle buttons for mechanism state
- Press/release edge detection
- Touchpad support on PS4/PS5 controllers
- Shared trigger thresholds and stick deadzones
- Rumble or LED feedback to drivers
- One wrapper managing both `gamepad1` and `gamepad2` while still exposing clear per-pad APIs

## Things to watch

- Avoid double-applying stick curves if the drive stack already shapes inputs.
- `tests/MecanumTeleOp.java` already applies its own deadzone and response curve inline, so do not layer `GamepadEx` stick curves on top of that path without removing one of the two shaping steps.
- Prefer one input style per op mode instead of mixing raw and wrapped reads unpredictably.
- If adding a wrapper feature, document whether it should replace or coexist with the repo’s current raw-FTC style.
- Keep subsystem construction in initialization through `Constants.create...(...)`, not in on-press handlers.
- Avoid awkward naming like `getTouchpadFinger21()` in new APIs; prefer explicit pad/finger ordering in method names.
