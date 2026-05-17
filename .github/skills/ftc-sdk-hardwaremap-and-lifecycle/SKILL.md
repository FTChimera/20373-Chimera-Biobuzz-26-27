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
- Mechanism constructors typically accept `HardwareMap` and resolve devices immediately.

## HardwareMap rules in this repo

- Resolve hardware once during initialization, not repeatedly inside the active loop.
- Prefer team wrappers such as `new Motor(map, name)` or `new Limelight(map, name)` over direct device access in op modes.
- Keep hardware names centralized in `Constants` or `library/robot/RobotConstants` depending on the device type.

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

## Common mistakes to avoid

- Re-fetching devices from `hardwareMap` in every loop.
- Writing FTC annotations on abstract base classes.
- Mixing `LinearOpMode` flow control into iterative `OpMode` subclasses.
- Forgetting that many runtime issues are hardware-config dependent even when the project builds.

