---
name: constants-not-configured
description: Use this skill when asked to fill in TeamCode constant values that are still placeholders because the final robot, field, or game data is not fully known yet.
---

# Constants Not Configured for this FTC Repo

Use this skill when needing to change constants that were left blank because of unknowns.

## Autonomous and Field Setup

- The Field Setup and scoring elements will be released September 2026.
- The Autonomouses under opModes and opModes/variants are all tests and they need to change once the BIOBUZZ game is released and the robot is fully built.
- The mechanisms used are drivetrain (MecanumDrive, Drivetrain, pedropathing Follower) and intake (Intake). However, once the scoring elements are released one or multiple subsystems need to be added.
- These will mostly be updated manually.

## Pedro Pathing Follower Configuration and Robot Constants

- The Preseason Robot (everything coded under TeamCode) hasn't completely been assembled as of mid-June 2026.
- Many mechanism params, follower values, and Pinpoint-related constants still have preseason placeholder values.
- When asked to update constants, update every relevant `*_PARAMS` owner (including wrappers under `library/hardware/...`) plus `RobotConstants` / `RobotConfigurations`, and do not forget any relevant op modes under `tests/`.

## Where those constants actually live

- `Constants.java` is the top-level op-mode-facing entry point. It holds:
  - `RED_DEFAULT_START_POSE`
  - `BLUE_DEFAULT_START_POSE`
  - `currentBot`, which currently points to `RobotConfigurations.OldDECODERobotTesting()`
  - factory helpers like `createFollower(...)`, `createTeleOpFollower(...)`, `createIntake(...)`, `createLimelight(...)`, `createDrivetrain(...)`, `createRGB(...)`, `createWebcam(...)`, and generic `createMechanism(...)`
  - mechanism-check helpers like `getMechanisms()`, `hasMechanism(...)`, and `assertHasMechanism(...)`
- `library/robot/RobotConfigurations.java` is the main place for active robot-profile values such as:
  - drive motor names and directions
  - `xVelocity(...)` and `yVelocity(...)`
  - `PinpointConstants`
  - Pedro follower constants and heading PIDF
  - path constraints and centripetal scaling
  - enabled mechanism registration via `configureDriveMechanisms()` and `.addMechanism(...)`
- `library/robot/RobotConstants.java` is the base profile object consumed by the factory helpers. If a constant affects follower construction or drivetrain hardware wiring, it usually belongs here or in `RobotConfigurations`.
- `library/hardware/hardwareDevice/` now contains custom FTC `HardwareDevice` definitions such as `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator`; these are lower-level SDK types, not profile-managed mechanisms.
- Mechanism-specific placeholder values still live inside the owning classes:
  - `mechanisms/Intake.java` -> `INTAKE_PARAMS`
  - `mechanisms/Slides1.java` -> `SLIDES1_PARAMS`
  - `library/hardware/webcam/Limelight.java` -> `LIMELIGHT_PARAMS`
  - `library/hardware/webcam/LogitechWebcam.java` -> `LOGITECH_WEBCAM_PARAMS`
  - `library/hardware/leds/RGBIndicator.java` -> `RGBIndicator_PARAMS`
- FTC AutoTune calibration values now also live in `pid/autoTune/TuningConstants.java`; those constants are for tuning op modes only and are not robot-profile-wide production defaults.
- Some preseason tuning constants still live directly inside op modes, especially `tests/PreseasonWebcamAuto.java`.

## Placeholder values that are real calibration targets in this repo

- `Constants.RED_DEFAULT_START_POSE` and `Constants.BLUE_DEFAULT_START_POSE` are both still `new Pose(0, 0, 0)`.
- `RobotConfigurations.OldDECODERobotTesting()` still uses placeholder drivetrain and follower values such as:
  - `.xVelocity(0)`
  - `.yVelocity(0)`
  - `.pinpointConstants(new PinpointConstants())`
  - `.headingPIDFCoefficients(0,0,0,0)`
  - `.centripetalScaling(0)`
- `Limelight.LIMELIGHT_PARAMS` still has `LIMELIGHT_MOUNT_ANGLE_DEGREES`, `LIMELIGHT_LENS_HEIGHT_INCHES`, and `TAG_HEIGHT_INCHES` at `0`.
- `LogitechWebcam.LOGITECH_WEBCAM_PARAMS` contains the default hardware name, overlay toggles, output units, and resolution that may need real robot-specific values.
- `RGBIndicator.RGBIndicator_PARAMS` contains the current LED hardware name and enablement gate.
- `Slides1.SLIDES1_PARAMS` contains the current slides hardware-map name and enablement gate.
- `Intake.INTAKE_PARAMS` contains intake power presets, motor direction, and hardware name defaults.
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

- `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` use `Constants.createIntake(...)`, `Constants.createTeleOpFollower(...)`, and `Constants.createLimelight(...)`.
- `opModes/IvyAuton.java` uses `Constants.createFollower(...)`, `Constants.createIntake(...)`, and the path starting pose.
- `tests/PedroPathingTeleOpTest.java` depends directly on `Constants.RED_DEFAULT_START_POSE` and `Constants.BLUE_DEFAULT_START_POSE`.
- `tests/MotorDirectionTest.java` depends on drivetrain motor directions and intake wiring.
- `opModes/variants/Red_TeleOp.java` and `Blue_TeleOp.java` return the default alliance start poses.
- `tests/PreseasonWebcamAuto.java` bypasses the Limelight wrapper for behavior, but now still sources the camera name from `Limelight.LIMELIGHT_PARAMS`; camera-name or vision-tuning updates may still need to touch it explicitly.
- `tests/MecanumTeleOp.java` now pulls drivetrain motor names and default directions from `Constants.currentBot`, so drive-profile renames should stay aligned automatically.
- `tests/ftcSdk/RGBIndicatorTest.java` and `tests/ftcSdk/SlidesMotorTest.java` now source hardware names from the owning params classes instead of stale standalone literals.
- `pid/autoTune/PositionPIDTunerOpMode.java`, `VelocityPIDFTunerOpMode.java`, and `DualVelocityPIDFTunerOpMode.java` depend directly on `pid/autoTune/TuningConstants.java` for motor names, reversal booleans, target speed/position, and relay-test settings.

## Repo-specific warning signs

- `Constants.currentBot` currently hardcodes `RobotConfigurations.OldDECODERobotTesting()`. If the user wants a different robot preset, changing nested constants alone is not enough.
- `RobotConstants.createMechanism(...)` throws if the mechanism class is not registered in `Mechanisms`. If a constant change introduces or enables a subsystem, also check `.addMechanism(...)`.
- `Constants.assertHasMechanism(...)` now also fails early on the convenience-factory path, so mechanism registration issues may surface before reflection-based construction.
- `MecanumDrive` and `Drivetrain` are normally enabled together through `configureDriveMechanisms()`, so drive constants and drive-stack registration need to stay coherent.
- `Limelight`, `LogitechWebcam`, and `RGBIndicator` toggle themselves off based on whether their class is present in `bot.Mechanisms`.
- `LogitechWebcam` settings are mostly owned by its local params class, not by `RobotConstants`.
- `Slides1` exists as a mechanism shell, but it is not currently registered in `RobotConfigurations.OldDECODERobotTesting()`, so changing its params does not automatically make it available through the normal `Constants.create...(...)` path.
- `pid/autoTune/` is intentionally not on the normal `Constants.create...(...)` mechanism-factory path; if the task is about those tuner op modes, update `TuningConstants` instead of trying to thread the values through `RobotConfigurations`.
- This codebase is still preseason-oriented, so some autonomous constants genuinely cannot be inferred until game rules, field layout, and the final robot hardware are known.

## Rules

- Prefer changing the active profile and existing params owners instead of inventing a new constants file.
- Keep drivetrain, follower, and localization constants under `RobotConfigurations` / `RobotConstants`.
- Keep per-wrapper values in the wrapper’s nested params class unless the repo already treats them as robot-profile-wide.
- Update all relevant tests and op modes that consume duplicated constants or raw hardware names.
- Do not blindly replace every `0`; only replace values that are acting as real placeholders for hardware, tuning, geometry, or field setup.
