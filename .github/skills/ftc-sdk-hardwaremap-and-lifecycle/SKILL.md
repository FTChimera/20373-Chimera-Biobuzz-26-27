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
- `opModes/IvyAuton.java` and `opModes/PreseasonWebcamAuto.java` use iterative `OpMode` lifecycle methods like `init()`, `start()`, `loop()`, and `stop()`.
- Mechanism and wrapper constructors typically accept `HardwareMap` plus a params object and resolve devices immediately.

## HardwareMap rules in this repo

- Resolve hardware once during initialization, not repeatedly inside the active loop.
- Prefer team wrappers and mechanisms over raw device access in op modes.
- Prefer `Constants.create...(...)` factories over direct `new Mechanism(...)` calls in op modes.
- Keep hardware names and editable subsystem params centralized in `Constants` unless they are drive/follower profile data that belong in `library/robot/RobotConstants`.

## New mechanism pattern tied to lifecycle

When a task asks for a new mechanism or wrapper:

1. define a nested params type in the mechanism or wrapper class
2. create one shared params instance in `Constants`
3. add `Constants.createMechanism(hardwareMap)`
4. initialize the mechanism once during `init()` or before `waitForStart()`

Do not create a fresh params object every loop. The team wants one central Dashboard-editable params instance in `Constants` and one constructed mechanism instance per op mode.

## Lifecycle rules

- In `LinearOpMode`, initialize first, then call `waitForStart()`, then loop while `opModeIsActive()`.
- In iterative `OpMode`, put one-time setup in `init()`, one-time start actions in `start()`, repeated work in `loop()`, and cleanup/persistence in `stop()`.
- Do not call `waitForStart()` from iterative `OpMode` classes.
- Stop or persist important state in `stop()` when the repo already relies on that pattern, such as pose handoff.

## FTC SDK details already used here

- The repo uses FTC annotation discovery with `@TeleOp` and `@Autonomous`.
- SDK gamepad edge helpers like `aWasPressed()` and `backWasPressed()` are already used in the codebase.
- `telemetry.addData(...)` plus `telemetry.update()` is the common reporting pattern.

## Good practice

- If you need a new selectable mode, create a concrete annotated class.
- If you need reusable lifecycle behavior, put it in an abstract base class and keep the annotation on the concrete subclass.
- Use `requestOpModeStop()` rather than custom termination flags when the op mode should end.
- If a subsystem is already factory-created from `Constants`, keep using that path instead of mixing direct constructors into one op mode.

## Common mistakes to avoid

- Re-fetching devices from `hardwareMap` in every loop.
- Writing FTC annotations on abstract base classes.
- Mixing `LinearOpMode` flow control into iterative `OpMode` subclasses.
- Building mechanism params ad hoc in multiple places instead of centralizing them in `Constants`.
- Forgetting that many runtime issues are hardware-config dependent even when the project builds.

