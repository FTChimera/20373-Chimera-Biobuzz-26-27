---
name: constants-not-configured
description: Use this skill when asked to fill in TeamCode constant values that are still placeholders because the final robot, field, or game data is not fully known yet.
---

# Constants Not Configured for this FTC Repo

Use this skill when needing to change constants that were left blank because of unknowns.

## Upstream `~/.gradle` lookup

If a constant or tuning field mirrors behavior defined by a dependency, inspect
the downloaded module under `~/.gradle/caches/modules-2/files-2.1/` using the
version pinned in `build.dependencies.gradle` before assuming the external API.

- `com.pedropathing/core` and `com.pedropathing/ftc` for Pedro motion and
  follower constants
- `com.pedropathing/ivy` for Ivy timing or scheduling assumptions
- `com.pedropathing/telemetry` for `pedroPathing/Tuning.java` and tuner
  selection issues
- `org.firstinspires.ftc/RobotCore`, `org.firstinspires.ftc/Hardware`, and
  sometimes `org.firstinspires.ftc/Vision` for FTC SDK-side defaults

## Autonomous and Field Setup

- The Field Setup and scoring elements will be released September 2026.
- The Autonomouses under opModes and opModes/variants are all tests and they need to change once the BIOBUZZ game is released and the robot is fully built.
- The only annotated autonomous variant currently under `opModes/variants/` is `Test_IvyAuton`; `IvyAuton` under `opModes/` is the reusable base class, and many of its path/pose defaults are still preseason placeholders.
- The `OldDECODERobotTesting` and `FirstRobot` profiles exist. `Constants.currentBot` selects the active profile; a class must be explicitly registered with `.addMechanism(...)` before its factory helper will create it.
- These will mostly be updated manually.

## Pedro Pathing Follower Configuration and Robot Constants

- Based on the BIOBUZZ preseason presentation, FirstRobot's drivetrain and
  Pinpoint hardware are finished; intake is planned to be added soon.
- FirstRobot has Pinpoint hardware name, pod offsets, and encoder directions
  configured. Do not overwrite those values as generic preseason placeholders.
- FirstRobot's Pedro drive-tuning constants are **not** configured yet:
  `X_VELOCITY` and `Y_VELOCITY` are `0`, `DRIVE_CONSTANTS` is a blank
  `FollowerConstants`, and heading PIDF / centripetal scaling are zero. Its
  motor names, directions, and max power are wiring defaults, not proof that
  follower-drive tuning is complete.
- Intake and other non-drive mechanism params still need real hardware
  integration before they are registered in FirstRobot.
- When asked to update constants, update every relevant `*_PARAMS` owner (including wrappers under `library/hardware/...`) plus `RobotConstants` / `RobotConfigurations`, and do not forget any relevant op modes under `tests/`.

## Where those constants actually live

- `Constants.java` is the top-level op-mode-facing entry point. It holds:
  - `RED_DEFAULT_START_POSE`
  - `BLUE_DEFAULT_START_POSE`
  - `currentBot`, which selects the active robot profile (assigned in `Constants.java`)
  - factory helpers like `createFollower(...)`, `createIMU(...)`, `createDrive(...)`, `createIntake(...)`, `createLimelight(...)`, `createRGB(...)`, `createWebcam(...)`, aggregate `createMechanisms(...)`, and generic `createMechanism(...)`
  - mechanism-check helpers like `getMechanisms()`, `hasMechanism(...)`, and `assertHasMechanism(...)`
- `library/robot/RobotConfigurations.java` is the main place for active robot-profile values such as:
  - drive motor names and directions
  - IMU hardware name plus `RevHubOrientationOnRobot` logo/USB directions
  - `xVelocity(...)` and `yVelocity(...)`
  - `PinpointConstants`
  - Pedro follower constants and heading PIDF
  - path constraints and centripetal scaling
  - enabled mechanism registration via `configureDriveMechanisms()` and `.addMechanism(...)`
- `library/robot/RobotConstants.java` is the base profile object consumed by the factory helpers. If a constant affects follower construction or drivetrain hardware wiring, it usually belongs here or in `RobotConfigurations`.
- `library/hardware/hardwareDevice/` now contains custom FTC `HardwareDevice` definitions such as `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator`; these are lower-level SDK types, not profile-managed mechanisms.
- Mechanism-specific placeholder values still live inside the owning classes as builder-style nested params:
  - `mechanisms/Intake.java` -> `INTAKE_PARAMS`
  - `library/hardware/webcam/Limelight.java` -> `LIMELIGHT_PARAMS`
  - `library/hardware/webcam/LogitechWebcam.java` -> `LOGITECH_WEBCAM_PARAMS`
  - `library/hardware/leds/RGBIndicator.java` -> `RGBIndicator_PARAMS`
- FTC AutoTune calibration values now also live in `pid/autoTune/TuningConstants.java`; those constants are for tuning op modes only and are not robot-profile-wide production defaults.
- Some preseason tuning constants still live directly inside op modes, especially `tests/PreseasonWebcamAuto.java`.

## Placeholder values that are real calibration targets in this repo

- `Constants.RED_DEFAULT_START_POSE` and `Constants.BLUE_DEFAULT_START_POSE` are both still `new Pose(0, 0, 0)`.
- Robot profiles contain follower-drive, localizer, and mechanism-registration configuration. FirstRobot's Pinpoint geometry is configured, but its follower-drive tuning remains a calibration target; inspect the selected profile before relying on a `Constants.create...(...)` helper as available.
- `Limelight.LIMELIGHT_PARAMS` still has `LIMELIGHT_MOUNT_ANGLE_DEGREES`, `LIMELIGHT_LENS_HEIGHT_INCHES`, and `TAG_HEIGHT_INCHES` at `0`.
- `LogitechWebcam.LOGITECH_WEBCAM_PARAMS` contains the default hardware name, overlay toggles, output units, and resolution that may need real robot-specific values.
- `RGBIndicator.RGBIndicator_PARAMS` contains the LED hardware name, panels name, indicator type (`ONLY_HARDWARE`/`ONLY_VIRTUAL`/`BOTH`/`NAN`), and enablement gate.
- `SlidesMotor.DEFAULT_HARDWAREMAP_NAME` contains the default hardware-map name used by the current direct SDK-level slides test.
- `Intake.INTAKE_PARAMS` contains intake power presets, motor direction, the intake motor hardware name, and the raw slides setup defaults (`HARDWAREMAP_NAME_SLIDES`, `SLIDES_MIN`, `SLIDES_MAX`).
- `tests/PreseasonWebcamAuto` contains local PIDF constants that are not sourced from `Constants` or `RobotConfigurations`.
- `pid/autoTune/TuningConstants.java` contains additional tune-target placeholders such as `MOTOR_NAME`, `MOTOR_NAME_2`, `TARGET_RPM`, `TICKS_PER_REV`, `RELAY_AMPLITUDE`, and relay/feedforward test settings.

## What is usually a real constant request versus a normal zero

- Usually update these when the user asks for missing constants:
  - hardware names
  - motor directions
  - robot start poses
  - Limelight mount geometry
  - follower tuning values
  - Pinpoint offsets / directions / resolution choices
  - path constraints and drive calibration
  - mechanism enablement for the active robot profile
- Usually do **not** treat these as missing configuration just because they are `0`:
  - stop powers like `STOP_POWER = 0.0`
  - temporary loop variables
  - reset values
  - idle vectors and PID controller state
  - math accumulators or indexing variables

## How to handle a constants task in this repo

1. Identify whether the requested value belongs in `Constants`, `RobotConfigurations`, `RobotConstants`, or a nested `*_PARAMS` class.
2. Update the active robot profile first if the value is robot-wide.
3. Update the owning mechanism params class if the value is specific to one wrapper or subsystem and is not currently profile-managed.
4. Search for tests or op modes that duplicate the same constant or bypass the wrapper and update those too, including `pid/autoTune/` when the change affects tuner motor names, target RPM, or relay-test settings.
5. If the change affects mechanism availability, update the `Mechanisms` set through `configureDriveMechanisms()` and/or `.addMechanism(...)`.
6. Keep the factory flow intact: op modes should still use `Constants.create...(...)` helpers rather than new ad hoc constructors.

## Op modes and tests that are especially affected by constant changes

- `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` call `Constants.createMechanisms(hardwareMap)`, then require `RobotDrive.class` and treat `Intake` / `Limelight` as optional aggregate lookups. With FirstRobot selected they can still run the drive stack, but intake and Limelight behavior stays absent until those classes are registered.
- `opModes/IvyAuton.java` uses `Constants.createFollower(...)` and `Constants.createIntake(...)`, but its starting pose comes from the selected `IvyAuton_Paths.getStartingPose()` implementation, not from the default alliance poses in `Constants`.
- `tests/PedroPathingTeleOpTest.java` depends directly on `Constants.RED_DEFAULT_START_POSE` and `Constants.BLUE_DEFAULT_START_POSE`.
- `tests/MotorDirectionTest.java` depends on drivetrain motor directions and intake wiring; it calls `Constants.createDrive(...)` and `Constants.createIntake(...)`, so it requires a profile that registers both `RobotDrive.class` and `Intake.class`.
- `opModes/variants/Red_TeleOp.java` and `Blue_TeleOp.java` first reuse `Blackboard.Caption.POSE` when it is non-zero, then fall back to the default alliance start poses.
- `tests/FieldCentricMecanumTeleOp.java` gets its IMU through `Constants.createIMU(hardwareMap)`, but its right-side motor directions are still hardcoded to `DcMotorSimple.Direction.REVERSE` rather than read from `Constants.currentBot`.
- `tests/PreseasonWebcamAuto.java` bypasses the Limelight wrapper for behavior, but now still sources the camera name from `Limelight.LIMELIGHT_PARAMS`; camera-name or vision-tuning updates may still need to touch it explicitly.
- `tests/MecanumTeleOp.java` now pulls drivetrain motor names and default directions from `Constants.currentBot`, so drive-profile renames should stay aligned automatically.
- `tests/ftcSdk/RGBIndicatorTest.java` and `tests/ftcSdk/SlidesMotorTest.java` now source hardware names from the owning wrapper/device definitions instead of stale standalone literals.
- `pid/autoTune/PositionPIDTunerOpMode.java`, `VelocityPIDFTunerOpMode.java`, and `DualVelocityPIDFTunerOpMode.java` depend directly on `pid/autoTune/TuningConstants.java` for motor names, reversal booleans, target speed/position, and relay-test settings.

## Repo-specific warning signs

- The active profile is set via `Constants.currentBot`. `configureDriveMechanisms()` currently registers `RobotDrive.class`; when FirstRobot is selected, `createIntake(...)`, `createLimelight(...)`, `createRGB(...)`, and `createWebcam(...)` still fail early until their classes are explicitly registered with `.addMechanism(...)`.
- `RobotConstants.createMechanism(...)` throws if the mechanism class is not registered in `Mechanisms`. If a constant change introduces or enables a subsystem, also check `.addMechanism(...)`.
- IMU settings are robot-profile-wide: update `IMU_HARDWAREMAP_NAME`, `IMU_LOGO_FACING_DIRECTION`, and `IMU_USB_FACING_DIRECTION` through the existing fluent builders, then use `Constants.createIMU(hardwareMap)` from op modes. The factory initializes orientation but intentionally does not reset yaw.
- `Constants.assertHasMechanism(...)` now also fails early on the convenience-factory path, so mechanism registration issues may surface before reflection-based construction.
- `RobotDrive` is the normal drive mechanism enabled through `configureDriveMechanisms()`, so drive constants and drive-stack registration need to stay coherent.
- A `Constants.create...(...)` helper can throw before a wrapper’s local `ISBEINGUSED` gate ever runs. With FirstRobot selected, every non-drive factory helper fails early because its class is absent from `bot.Mechanisms`.
- `Limelight`, `LogitechWebcam`, and `RGBIndicator` do still toggle themselves off when constructed with a profile that omits their class, but the normal convenience-factory path now usually hits `assertHasMechanism(...)` first.
- `LogitechWebcam` settings are mostly owned by its local params class, not by `RobotConstants`.
- `pid/autoTune/` is intentionally not on the normal `Constants.create...(...)` mechanism-factory path; if the task is about those tuner op modes, update `TuningConstants` instead of trying to thread the values through `RobotConfigurations`.
- This codebase is still preseason-oriented, so some autonomous constants genuinely cannot be inferred until game rules, field layout, and the final robot hardware are known.

## Rules

- Prefer changing the active profile and existing params owners instead of inventing a new constants file.
- Keep drivetrain, follower, and localization constants under `RobotConfigurations` / `RobotConstants`.
- Keep per-wrapper values in the wrapper’s nested params class unless the repo already treats them as robot-profile-wide.
- Update all relevant tests and op modes that consume duplicated constants or raw hardware names.
- Do not blindly replace every `0`; only replace values that are acting as real placeholders for hardware, tuning, geometry, or field setup.
