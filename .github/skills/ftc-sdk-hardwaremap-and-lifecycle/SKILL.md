---
name: ftc-sdk-hardwaremap-and-lifecycle
description: Use this skill when asked to call FTC SDK APIs directly, work with `HardwareMap`, write `LinearOpMode` or `OpMode` classes, or debug lifecycle ordering in this repo.
---

# FTC SDK HardwareMap and Lifecycle

Use this skill for direct FTC SDK integration work.

## Repo context

- The project depends on FTC SDK `11.1.0`.
- Team code lives in `TeamCode/`; SDK app code and samples live in `FtcRobotController/`.
- This repo uses both `LinearOpMode` and iterative `OpMode` styles.

## Current examples to follow

- `opModes/TeleOp.java` uses `LinearOpMode` with `waitForStart()` and `while (opModeIsActive())`.
- `opModes/IvyTeleOp.java` uses `LinearOpMode` plus Ivy scheduling.
- `opModes/IvyAuton.java` uses iterative `OpMode` lifecycle methods like `init()`, `start()`, `loop()`, and `stop()`.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept `LinearOpMode` that mixes raw Limelight SDK access with a follower control loop.
- `configs/ConfigOpMode.java` is a one-shot iterative config helper that runs a `Runnable` in `start()` and then requests stop.
- `tests/ftcSdk/androidTest.java` is a shared `LinearOpMode` base for simple SDK-level validation flows.
- `tests/MecanumTeleOp.java` is a current iterative `OpMode` example that resolves raw SDK devices in `init()`, uses `init_loop()` for mode selection, and handles IMU reset plus drive updates in `loop()`.
- Profile-managed mechanism and wrapper constructors typically accept `HardwareMap` plus `RobotConstants` and resolve devices immediately.

## HardwareMap rules in this repo

- Resolve hardware once during initialization, not repeatedly inside the active loop.
- Prefer team wrappers and mechanisms over raw device access in op modes.
- Prefer `Constants.create...(...)` factories over direct `new Mechanism(...)` calls in op modes.
- Keep drive/profile hardware names in `RobotConstants`, and let `Constants.create...(...)` delegate to the active profile for profile-managed subsystems.
- If a wrapper can be disabled by robot profile, make sure it decides that before calling `hardwareMap.get(...)`.

## New mechanism pattern tied to lifecycle

When a task asks for a new mechanism or wrapper:

1. make it extend `Mechanism` if it belongs in the standard robot subsystem flow
2. give it the `(HardwareMap, RobotConstants)` constructor that `RobotConstants.createMechanism(...)` expects
3. register it in the active robot profile with `.addMechanism(...)`
4. add or reuse a `Constants.create...(...)` helper when higher-level code should access it directly
5. initialize the mechanism once during `init()` or before `waitForStart()`

Do not create fresh subsystem instances every loop. The team wants one constructed mechanism instance per op mode, created through the normal profile-aware factory path.

## Lifecycle rules

- In `LinearOpMode`, initialize first, then call `waitForStart()`, then loop while `opModeIsActive()`.
- In iterative `OpMode`, put one-time setup in `init()`, one-time start actions in `start()`, repeated work in `loop()`, and cleanup/persistence in `stop()`.
- Do not call `waitForStart()` from iterative `OpMode` classes.
- Stop or persist important state in `stop()` when the repo already relies on that pattern, such as pose handoff.

## FTC SDK details already used here

- The repo uses FTC annotation discovery with `@TeleOp` and `@Autonomous`.
- SDK gamepad edge helpers like `aWasPressed()`, `backWasPressed()`, `dpadLeftWasPressed()`, and `dpadRightWasPressed()` are already used in the codebase.
- `telemetry.addData(...)` plus `telemetry.update()` is the common reporting pattern.
- Kotlin op modes are also part of the TeamCode module now, so lifecycle/build work should tolerate both Java and Kotlin sources.

## Good practice

- If you need a new selectable mode, create a concrete annotated class.
- If you need reusable lifecycle behavior, put it in an abstract base class and keep the annotation on the concrete subclass.
- Use `requestOpModeStop()` rather than custom termination flags when the op mode should end.
- If a subsystem is already factory-created from `Constants`, keep using that path instead of mixing direct constructors into one op mode.
- Use `configs/ConfigOpMode` for one-shot configuration mutations rather than re-implementing that pattern in each config class.

## Common mistakes to avoid

- Re-fetching devices from `hardwareMap` in every loop.
- Writing FTC annotations on abstract base classes.
- Mixing `LinearOpMode` flow control into iterative `OpMode` subclasses.
- Bypassing the `Mechanism` / `RobotConstants.createMechanism(...)` pattern for profile-managed subsystems.
- Forgetting that many runtime issues are hardware-config dependent even when the project builds.
- Forgetting that `tests/` may intentionally contain proof-of-concept code that uses more direct SDK access than shared match op modes.

