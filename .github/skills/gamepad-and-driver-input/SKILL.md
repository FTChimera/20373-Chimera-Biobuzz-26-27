---
name: gamepad-and-driver-input
description: Use this skill when asked to work on driver controls, raw FTC gamepad calls, or the repo’s `GamepadEx` / button-reader abstractions.
---

# Gamepad and Driver Input

This skill covers both the raw FTC gamepad APIs already used in TeamCode and the custom wrapper utilities in `library/hardware/gamepad`.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/GamepadEx.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Buttons.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Reader.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Stick.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/Trigger.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`

## Current repo usage

- Most active op modes use raw SDK gamepads like `gamepad1.left_stick_y`, `gamepad1.back`, or edge helpers such as `aWasPressed()`.
- `GamepadEx` exists as a richer wrapper with buttons, sticks, triggers, touchpad, LED, and rumble support.
- `GamepadEx` is marked `@Deprecated`, so treat it as optional rather than the default style for new work.

## Rules

- If staying consistent with existing op modes, raw FTC gamepad access is acceptable.
- If you need reusable input behavior like toggles, edge detection, touchpad tracking, or controller LED support, extend the wrapper utilities instead of recreating them ad hoc.
- Call wrapper `update()` methods once per loop before reading values.
- Keep stick-curve logic consistent with drive expectations; the repo often applies shaping in `MecanumDrive` / `DriveVector`, not directly in every button handler.

## Good use cases for wrappers

- Toggle buttons for mechanism state
- Press/release edge detection
- Touchpad support on PS4/PS5 controllers
- Shared trigger thresholds and stick deadzones
- Rumble or LED feedback to drivers

## Things to watch

- Avoid double-applying stick curves if the drive stack already shapes inputs.
- Prefer one input style per op mode instead of mixing raw and wrapped reads unpredictably.
- If adding a wrapper feature, document whether it should replace or coexist with the repo’s current raw-FTC style.

