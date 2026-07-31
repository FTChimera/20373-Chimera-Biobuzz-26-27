---
name: teamcode-library-architecture
description: Use this skill when asked to work with the team's current library, hardware wrappers, mechanisms, constants, or overall TeamCode architecture instead of generic FTC examples.
---

# TeamCode Library Architecture

Use this skill to stay aligned with the repo’s current architecture instead of falling back to generic FTC examples.

## Package layout

- `configs/`: `Blackboard` plus one-shot configuration op modes such as `ZeroPose`, `ResetAllBlackboard`, and `Set*Driver`.
- `library/`: small reusable types and wrappers, now split across areas such as `control/`, `filters/`, `hardware/`, `interpolation/`, `math/`, and `robot/`.
- `library/hardware/hardwareDevice/`: SDK-registerable custom devices such as `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator`.
- `mechanisms/`: robot subsystems plus the shared abstract base class `Mechanism`.
- `library/OpModeHelper.kt`: optional op-mode utility for manual Lynx bulk caching and loop `dt` tracking.
- `opModes/`: shared base op mode logic.
- `opModes/variants/`: thin FTC-annotated wrappers such as `Blue_TeleOp`, `Red_TeleOp`, and `Test_IvyAuton`.
- `pid/autoTune/`: FTC AutoTune relay-feedback tuner op modes, repo-local FTC master adapters, and a shared `TuningConstants` config class for calibration-only workflows.
- `pedroPathing/`: Pedro-specific helpers, Ivy adapters, tuning entry points, and custom geometry such as `custom/EquationCurve`.
- `tests/`: driver-station validation and proof-of-concept op modes, including `PreseasonWebcamAuto` and `tests/ftcSdk` helpers.
- `learning/`: teaching or scratch op modes, including the Kotlin teleop example.
- `learning/flyset/`: an unannotated PIDF teaching sketch with placeholder
  `Sensor` methods; it is not hardware-ready robot code.
- `java-autogen/`: browser-side HTML/JS generators for producing starter Java autonomous classes from Pedro/Turtle Tracer path data or manual path-builder drafts.
- `Constants.java`: the active robot profile, default poses, and op-mode-facing convenience factories.

## House style for mechanisms and wrappers

This repo now wants reusable subsystems and higher-level wrappers to share one construction shape instead of ad hoc constructors in op modes.

Use this pattern:

1. Reusable robot subsystems and wrappers that depend on the active robot profile should extend `mechanisms/Mechanism`.
2. Give them a constructor shaped like `(HardwareMap, RobotConstants)`.
3. Use the `bot` profile passed into that constructor for shared robot-profile data and mechanism enablement checks.
4. Register optional subsystems in each robot preset with `RobotConstants.configureDriveMechanisms()` and/or `RobotConstants.addMechanism(...)`.
5. If the mechanism has local configuration, keep it in a nested builder-style params class that implements `Mechanism.Params<...>` and resolve it through `Mechanism.resolveParams(...)`.
6. Instantiate from `RobotConstants.createMechanism(...)`, usually through a small convenience helper in `Constants` such as `createIntake(hardwareMap)`.
7. Op modes should call the existing `Constants.create...(...)` helpers instead of using direct constructors.

Mechanisms with local config now own nested builder-style params types such as `Intake.INTAKE_PARAMS`, `Drivetrain.DRIVETRAIN_PARAMS`, `MecanumDrive.MECANUM_DRIVE_PARAMS`, `Limelight.LIMELIGHT_PARAMS`, `LogitechWebcam.LOGITECH_WEBCAM_PARAMS`, `RGBIndicator.RGBIndicator_PARAMS`, or `Slides1.SLIDES1_PARAMS`, and the uniform repo-level construction pattern is now `Mechanism` + `Mechanism.Params` + `RobotConstants.createMechanism(...)`.

The `OldDECODERobotTesting` and `FirstRobot` profiles exist. FirstRobot
registers only `MecanumDrive` and `Drivetrain`; its Pinpoint
geometry/directions are configured, but its Pedro drive-tuning fields are still
defaults and its planned intake is not yet registered. A helper existing in
`Constants` does **not** mean the profile selected by `Constants.currentBot`
registers that class; check its `Mechanisms` set before using a factory helper.

## Where configuration belongs

- Put drive motor names, `Direction`-based drive directions, IMU names and hub orientation, Pinpoint data, follower tuning, path constraints, and the enabled-mechanism set in `library/robot/RobotConstants` / `RobotConfigurations`.
- Put default poses and top-level convenience factories in `Constants`, including `createIMU(...)`, generic helpers such as `createMechanism(...)`, `getMechanisms()`, `hasMechanism(...)`, and mechanism-specific helpers like `createRGB(...)`, `createWebcam(...)`, and `createSlides1(...)`.
- Keep mechanism-specific behavior and optional nested params types in the mechanism or wrapper class that owns them.
- Keep lightweight math helpers such as `Equation`, `Polynomial`, interpolation helpers, filters, and generic PIDF controllers in `library/`.
- Keep calibration-only tuner adapters such as `pid/autoTune/*` in their own package; they are op-mode utilities, not profile-managed mechanisms.
- Keep Pedro-specific curve integration such as `EquationCurve` in `pedroPathing/custom/`.
- Do not turn `RobotConstants` into a catch-all for arbitrary per-mechanism tuning unless that data is truly robot-profile-wide.

## Existing wrappers and mechanisms to reuse

- `library.hardware.actuators.Direction`
- `library.hardware.actuators.Motor`
- `library.hardware.hardwareDevice.RevPotentiometer`
- `library.hardware.hardwareDevice.SlidesMotor`
- `library.hardware.hardwareDevice.RGBIndicator`
- `library.hardware.leds.RGBIndicator`
- `library.hardware.webcam.Limelight`
- `library.hardware.webcam.LogitechWebcam`
- `library.DriveVector`
- `library.control.PIDFController`
- `library.control.DoublePIDFController`
- `library.interpolation.LinearInterp`
- `library.interpolation.LagrangeInterp`
- `library.math.Equation`
- `library.math.Polynomial`
- `library.math.Term`
- `mechanisms.Intake`
- `mechanisms.Drivetrain`
- `mechanisms.MecanumDrive`
- `mechanisms.Slides1`
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
- Use `Constants.createIMU(hardwareMap)` for profile-configured IMUs. It initializes the active profile's `RevHubOrientationOnRobot` settings but never calls `resetYaw()`; callers own their heading-reset timing.
- Keep pose and driver-preference transfer through `configs/Blackboard` when op modes need shared state.
- Match the repo’s split between mechanism logic, wrapper logic, and op mode orchestration.

## Good places to edit for common tasks

- New top-level subsystem convenience factory or active-profile selection: `Constants.java`
- New drive profile or Pedro follower constant set: `library/robot/...`
- New simple hardware wrapper: `library/hardware/...`
- New custom SDK hardware device definition: `library/hardware/hardwareDevice/...`
- New subsystem behavior: `mechanisms/...`
- New shared blackboard/configuration preference: `configs/...`
- New teleop or autonomous entry: `opModes/variants/...` or `tests/...`, depending on whether it is match logic or validation logic
- New Pedro/localization behavior: `pedroPathing/...`
- New FTC AutoTune calibration wrapper or repo-local tuner constants: `pid/autoTune/...`
- New lightweight math/filter/interpolation helper: `library/...`

## Things not to assume

- Generic FTC sample patterns from `FtcRobotController/external/samples/` are not the house style for TeamCode.
- New mechanisms should not be instantiated directly from op modes when a `Constants.create...(...)` factory belongs there.
- A profile-managed `Constants.create...(...)` helper is only a convenience surface; it still depends on the active profile’s `Mechanisms` set and `Constants.assertHasMechanism(...)`. `createFollower(...)` and `createIMU(...)` use the active profile without requiring a registered mechanism.
- A mechanism no longer needs a centrally shared params singleton in `Constants` to fit repo style; the important uniformity point is the `Mechanism` + `RobotConstants.createMechanism(...)` construction flow.
- `Polynomial` now lives under `library/math/`, implements the newer `Equation` interface, and is still the only currently usable reusable `Equation` implementation for evaluation/derivative work.
- `Term` currently exists as a scaffold for the broader `Equation` abstraction; do not assume symbolic equation algebra is production-ready yet.
- `EquationCurve` is the Pedro-specific adapter that turns repo equation geometry into a `CustomCurve`, now with an explicit `HeadingInterpolator`.
- `tests/` and `learning/` intentionally contain validation and teaching code; not every file there is meant to model final match architecture.
- `java-autogen/` is intentional tooling, but it is not part of the Android/FTC runtime architecture until generated code is copied into `TeamCode/` and reviewed.
- `pid/autoTune/` is also intentional calibration code: those op modes may use direct motor names from `TuningConstants` because repo-local `PIDMaster` / `DualMotorPIDMaster` classes own the FTC relay-test layer, so do not force that package into the normal `Constants.create...(...)` mechanism-factory shape.
- The repo now has both a device-level `library.hardware.hardwareDevice.RGBIndicator` and a profile-managed wrapper `library.hardware.leds.RGBIndicator`; use the former for SDK registration tests and the latter for normal mechanism/factory-driven robot code.

## Current build-tooling note

- `TeamCode/build.gradle` wires Project Lombok annotation processing, Sloth load/runtime support, and the Kotlin Android plugin.
- `build.dependencies.gradle` now wires JitPack plus `com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.3.7`; the repo-local `pid/autoTune/master/` classes supply the removed FTC-specific adapter layer.
- The root `build.gradle` currently pins the Kotlin Gradle plugin with `kotlin_version = '2.1.20'`, so build or architecture work should preserve that wiring unless the repo is intentionally being migrated.
