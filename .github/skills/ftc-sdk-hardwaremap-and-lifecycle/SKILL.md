---
name: ftc-sdk-hardwaremap-and-lifecycle
description: Use this skill when asked to call FTC SDK APIs directly, work with `HardwareMap`, write `LinearOpMode` or `OpMode` classes, or debug lifecycle ordering in this repo.
---

# FTC SDK HardwareMap and Lifecycle

Use this skill for direct FTC SDK integration work.

## Upstream `~/.gradle` lookup

When TeamCode wraps an FTC SDK behavior, inspect the downloaded SDK modules
under `~/.gradle/caches/modules-2/files-2.1/` using the version pinned in
`build.dependencies.gradle`.

- `org.firstinspires.ftc/RobotCore` for op mode, telemetry, gamepad, and core
  hardware-map behavior
- `org.firstinspires.ftc/Hardware` for controllers, motors, sensors, and
  hardware wrappers
- `org.firstinspires.ftc/Vision` when AprilTag, camera, or vision-portal APIs
  are involved
- `com.pedropathing/telemetry` if the issue is actually tied to
  `pedroPathing/Tuning.java` op-mode selection rather than base SDK lifecycle

## Repo context

- The project depends on FTC SDK `11.2.1`.
- Team code lives in `TeamCode/`; SDK app code and samples live in `FtcRobotController/`.
- This repo uses both `LinearOpMode` and iterative `OpMode` styles.
- The `OldDECODERobotTesting` and `FirstRobot` profiles exist. `Constants.currentBot` selects the profile used for profile-managed factories; always verify the selected profile's registrations before calling an optional mechanism factory.
- The repo now also contains custom FTC `HardwareDevice` implementations under `library/hardware/hardwareDevice/` for direct `hardwareMap.get(CustomType.class, ...)` usage.

## Current examples to follow

- `opModes/TeleOp.java` uses `LinearOpMode` with `waitForStart()` and `while (opModeIsActive())`.
- `opModes/IvyTeleOp.java` uses `LinearOpMode` plus Ivy scheduling.
- `opModes/IvyAuton.java` uses iterative `OpMode` lifecycle methods like `init()`, `start()`, `loop()`, and `stop()`.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept `LinearOpMode` that mixes raw Limelight SDK access with a follower control loop.
- `configs/ConfigOpMode.java` is an iterative pre-start configuration menu. `init()` builds the available blackboard mutations, `init_loop()` handles the D-pad selection UI, and `loop()` immediately requests stop after the driver starts the op mode.
- `tests/ftcSdk/androidTest.java` is a shared `LinearOpMode` base for simple SDK-level validation flows that run one scripted test, sleep briefly, and then request stop.
- `tests/MecanumTeleOp.java` is a current iterative `OpMode` example that resolves raw SDK devices in `init()`, uses `init_loop()` for mode selection, and handles IMU reset plus drive updates in `loop()`.
- `Constants.createIMU(hardwareMap)` resolves the active profile's IMU, initializes it with the profile's REV Hub logo/USB orientation, and intentionally leaves yaw unchanged.
- `library/OpModeHelper.kt` is an optional utility that wraps manual Lynx bulk caching and loop-`dt` tracking across `init()`, `start()`, `update()`, and `close()` style calls.
- `pid/autoTune/PositionPIDTunerOpMode.java`, `VelocityPIDFTunerOpMode.java`, and `DualVelocityPIDFTunerOpMode.java` are `LinearOpMode` calibration wrappers around repo-local `PIDMaster` / `DualMotorPIDMaster` FTC adapters, backed by the external `pidautotuner-core` algorithms.
- Profile-managed mechanism and wrapper constructors typically accept `HardwareMap` plus `RobotConstants` and resolve devices immediately.
- `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` show the intended aggregate factory style: they call `Constants.createMechanisms(hardwareMap)`, require `RobotDrive`, and treat `Intake` / `Limelight` as optional aggregate lookups.

## HardwareMap rules in this repo

- Resolve hardware once during initialization, not repeatedly inside the active loop.
- Prefer team wrappers and mechanisms over raw device access in op modes.
- Prefer `Constants.create...(...)` factories over direct `new Mechanism(...)` calls in op modes.
- Every profile-managed `Constants.create...(...)` helper enforces `Constants.assertHasMechanism(...)` first; if the active profile has not registered that class, the helper throws before any device lookup happens. `createFollower(...)` and `createIMU(...)` are profile-configured helpers, not mechanism factories.
- Keep drive/profile hardware names in `RobotConstants`, and let `Constants.create...(...)` delegate to the active profile for profile-managed subsystems.
- Keep `IMU_HARDWAREMAP_NAME`, `IMU_LOGO_FACING_DIRECTION`, and `IMU_USB_FACING_DIRECTION` in `RobotConstants`; use `Constants.createIMU(hardwareMap)` instead of duplicating IMU lookup and orientation code.
- If a wrapper can be disabled by robot profile, make sure it decides that before calling `hardwareMap.get(...)`.
- If a class is a custom FTC `HardwareDevice` like `SlidesMotor`, `RevPotentiometer`, or the device-level `RGBIndicator`, fetch it directly from `hardwareMap` and keep it distinct from the profile-managed `Mechanism` wrappers.
- `pid/autoTune/` is an intentional direct-motor exception: those calibration op modes pass motor names, directions, and target values through `TuningConstants` into `PIDMaster` / `DualMotorPIDMaster` instead of going through `Constants.create...(...)`.
- `Constants.create...(...)` is runtime-valid only when the profile selected by `Constants.currentBot` registers the requested mechanism class.

## New mechanism pattern tied to lifecycle

When a task asks for a new mechanism or wrapper:

1. make it extend `Mechanism` if it belongs in the standard robot subsystem flow
2. give it the `(HardwareMap, RobotConstants)` constructor that `RobotConstants.createMechanism(...)` expects
3. register it in the active robot profile with `.addMechanism(...)`
4. add or reuse a `Constants.create...(...)` helper when higher-level code should access it directly
5. initialize the mechanism once during `init()` or before `waitForStart()`

Do not create fresh subsystem instances every loop. The team wants one constructed mechanism instance per op mode, created through the normal profile-aware factory path.

If the task is instead about a custom SDK device type, put that class under `library/hardware/hardwareDevice/` and use normal FTC constructor/annotation rules rather than the `Mechanism` factory flow.

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
- The current `tests/ftcSdk/androidTest.java` base is an example of the preferred scripted-test termination pattern: run the validation, pause briefly if needed, then `requestOpModeStop()`.
- If a subsystem is already factory-created from `Constants`, keep using that path and update the active `RobotConstants` profile so the helper is actually registered instead of mixing direct constructors into one op mode.
- Use `configs/ConfigOpMode` for one-shot configuration mutations rather than re-implementing that pattern in each config class.
- If a proof-of-concept or SDK-level test bypasses the wrapper path, still source hardware names from `RobotConfigurations` or the owning `*_PARAMS` class when those names already exist.
- Reset IMU yaw explicitly in the op mode only when its control flow needs a new field-centric reference; `Constants.createIMU(...)` never resets it.

## Common mistakes to avoid

- Re-fetching devices from `hardwareMap` in every loop.
- Writing FTC annotations on abstract base classes.
- Mixing `LinearOpMode` flow control into iterative `OpMode` subclasses.
- Bypassing the `Mechanism` / `RobotConstants.createMechanism(...)` pattern for profile-managed subsystems.
- Confusing the device-level `library.hardware.hardwareDevice.RGBIndicator` with the profile-managed `library.hardware.leds.RGBIndicator` wrapper.
- Forgetting that many runtime issues are hardware-config dependent even when the project builds.
- Forgetting that `tests/` may intentionally contain proof-of-concept code that uses more direct SDK access than shared match op modes.
