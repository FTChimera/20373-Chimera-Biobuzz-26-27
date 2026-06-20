---
name: teamcode-library-architecture
description: Use this skill when asked to work with the team's current library, hardware wrappers, mechanisms, constants, or overall TeamCode architecture instead of generic FTC examples.
---

# TeamCode Library Architecture

Use this skill to stay aligned with the repo’s current architecture instead of falling back to generic FTC examples.

## Package layout

- `configs/`: `Blackboard` plus one-shot configuration op modes such as `ZeroPose`, `ResetAllBlackboard`, and `Set*Driver`.
- `library/`: small reusable types and wrappers, now split across areas such as `control/`, `filters/`, `hardware/`, `interpolation/`, `math/`, and `robot/`.
- `mechanisms/`: robot subsystems plus the shared abstract base class `Mechanism`.
- `opModes/`: shared base op mode logic.
- `opModes/variants/`: thin FTC-annotated wrappers such as `Blue_TeleOp`, `Red_TeleOp`, and `Test_IvyAuton`.
- `pedroPathing/`: Pedro-specific helpers, Ivy adapters, tuning entry points, and custom geometry such as `custom/EquationCurve`.
- `tests/`: driver-station validation and proof-of-concept op modes, including `PreseasonWebcamAuto` and `tests/ftcSdk` helpers.
- `learning/`: teaching or scratch op modes, including the Kotlin teleop example.
- `Constants.java`: the active robot profile, default poses, and op-mode-facing convenience factories.

## House style for mechanisms and wrappers

This repo now wants reusable subsystems and higher-level wrappers to share one construction shape instead of ad hoc constructors in op modes.

Use this pattern:

1. Reusable robot subsystems and wrappers that depend on the active robot profile should extend `mechanisms/Mechanism`.
2. Give them a constructor shaped like `(HardwareMap, RobotConstants)`.
3. Use the `bot` profile passed into that constructor for shared robot-profile data and mechanism enablement checks.
4. Register optional subsystems in each robot preset with `RobotConstants.configureDriveMechanisms()` and/or `RobotConstants.addMechanism(...)`.
5. Instantiate from `RobotConstants.createMechanism(...)`, usually through a small convenience helper in `Constants` such as `createIntake(hardwareMap)`.
6. Op modes should call the existing `Constants.create...(...)` helpers instead of using direct constructors.

Some mechanisms still own nested params types such as `Intake.INTAKE_PARAMS`, `Limelight.LIMELIGHT_PARAMS`, `LogitechWebcam.LOGITECH_WEBCAM_PARAMS`, or `RGBIndicator.RGBIndicator_PARAMS`, but the uniform repo-level construction pattern is now based on `Mechanism` plus `RobotConstants.createMechanism(...)`.

## Where configuration belongs

- Put drive motor names, `Direction`-based drive directions, Pinpoint data, follower tuning, path constraints, and the enabled-mechanism set in `library/robot/RobotConstants` / `RobotConfigurations`.
- Put default poses and top-level convenience factories in `Constants`, including generic helpers such as `createMechanism(...)`, `getMechanisms()`, `hasMechanism(...)`, and mechanism-specific helpers like `createRGB(...)` and `createWebcam(...)`.
- Keep mechanism-specific behavior and optional nested params types in the mechanism or wrapper class that owns them.
- Keep lightweight math helpers such as `Polynomial`, interpolation helpers, filters, and generic PIDF controllers in `library/`.
- Keep Pedro-specific curve integration such as `EquationCurve` in `pedroPathing/custom/`.
- Do not turn `RobotConstants` into a catch-all for arbitrary per-mechanism tuning unless that data is truly robot-profile-wide.

## Existing wrappers and mechanisms to reuse

- `library.hardware.actuators.Direction`
- `library.hardware.actuators.Motor`
- `library.hardware.leds.RGBIndicator`
- `library.hardware.webcam.Limelight`
- `library.hardware.webcam.LogitechWebcam`
- `library.DriveVector`
- `library.control.PIDFController`
- `library.control.DoublePIDFController`
- `library.interpolation.LinearInterp`
- `library.interpolation.LagrangeInterp`
- `library.math.Polynomial`
- `mechanisms.Intake`
- `mechanisms.Drivetrain`
- `mechanisms.MecanumDrive`
- `mechanisms.Mechanism`
- `library.robot.RobotConstants`
- `library.robot.RobotConfigurations`
- `configs.Blackboard`
- `pedroPathing.custom.EquationCurve`

## Rules

- Prefer extending existing wrappers over bypassing them.
- When adding a new profile-managed mechanism, extend `Mechanism`, use the `(HardwareMap, RobotConstants)` constructor shape, and register it in `RobotConfigurations`.
- Add a `Constants.create...(...)` helper when that mechanism is expected to be used directly by op modes or other high-level code.
- Keep op modes focused on orchestration, not low-level hardware construction.
- Keep pose and driver-preference transfer through `configs/Blackboard` when op modes need shared state.
- Match the repo’s split between mechanism logic, wrapper logic, and op mode orchestration.

## Good places to edit for common tasks

- New top-level subsystem convenience factory or active-profile selection: `Constants.java`
- New drive profile or Pedro follower constant set: `library/robot/...`
- New simple hardware wrapper: `library/hardware/...`
- New subsystem behavior: `mechanisms/...`
- New shared blackboard/configuration preference: `configs/...`
- New teleop or autonomous entry: `opModes/variants/...` or `tests/...`, depending on whether it is match logic or validation logic
- New Pedro/localization behavior: `pedroPathing/...`
- New lightweight math/filter/interpolation helper: `library/...`

## Things not to assume

- Generic FTC sample patterns from `FtcRobotController/external/samples/` are not the house style for TeamCode.
- New mechanisms should not be instantiated directly from op modes when a `Constants.create...(...)` factory belongs there.
- A mechanism no longer needs a centrally shared params singleton in `Constants` to fit repo style; the important uniformity point is the `Mechanism` + `RobotConstants.createMechanism(...)` construction flow.
- `Polynomial` now lives under `library/math/`, while `EquationCurve` is the Pedro-specific adapter that turns polynomial geometry into a `CustomCurve`.
- `tests/` and `learning/` intentionally contain validation and teaching code; not every file there is meant to model final match architecture.

## Current build-tooling note

- `TeamCode/build.gradle` wires Project Lombok annotation processing, Sloth load/runtime support, and the Kotlin Android plugin.
- The root `build.gradle` currently pins the Kotlin Gradle plugin with `kotlin_version = '2.1.20'`, so build or architecture work should preserve that wiring unless the repo is intentionally being migrated.

