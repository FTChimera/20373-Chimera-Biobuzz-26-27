---
name: robot-configurations-and-follower-builder
description: Use this skill when asked to edit robot profiles, drivetrain naming/directions, Pinpoint offsets, or Pedro follower construction in `Constants`, `RobotConfigurations`, or `RobotConstants`.
---

# Robot Configurations and Follower Builder

Use this skill for robot-profile work centered around:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`

## How this repo is wired

1. `Constants.currentBot` selects the active robot profile.
2. `RobotConfigurations` currently defines named robot presets `OldDECODERobotTesting()` and `PreseasonDrivetrain()`.
3. `RobotConstants` is a builder-style container for drive names, drive directions, IMU configuration, Pinpoint constants, follower tuning, path constraints, and the set of enabled `Mechanism` classes.
4. `Constants.createFollower(hardwareMap)` delegates follower creation to the active `RobotConstants` instance.
5. `RobotConstants.createFollower(...)` builds the Pedro follower through `FollowerBuilder` using `FollowerConstants`, `MecanumConstants`, `PinpointConstants`, and `PathConstraints`.
6. Drive motor directions in `RobotConstants` use the shared `library.hardware.actuators.Direction` enum and convert to FTC SDK directions only when building drivetrain/follower objects.
7. `RobotConstants.createMechanism(...)` is the uniform creation path for registered mechanisms and wrappers that follow the `(HardwareMap, RobotConstants)` constructor pattern.
8. `Constants` now also exposes helper methods such as `createIMU(...)`, `createRGB(...)`, `createWebcam(...)`, generic `createMechanism(...)`, `getMechanisms()`, `hasMechanism(...)`, and `assertHasMechanism(...)`.
9. `Constants.currentBot` currently selects `PreseasonDrivetrain()`, which only registers `MecanumDrive` and `Drivetrain` through `configureDriveMechanisms()` and sets `.useFollowerDrive(false)`, so teleop `MecanumDrive` currently runs its manual-drive path unless you switch or edit the profile.

## What belongs where

- Put drive motor names, drive directions, IMU hardware/orientation settings, follower/pathing tuning, and enabled mechanism classes in `RobotConstants`.
- Put concrete robot presets in `RobotConfigurations`.
- Put top-level convenience helpers such as `createIMU(...)`, `createIntake(...)`, `createLimelight(...)`, `createRGB(...)`, and `createWebcam(...)` in `Constants` when they make op-mode code cleaner.
- Keep mechanism-specific behavior and optional nested params types in the owning mechanism or wrapper class.
- Prefer selecting a profile through `Constants.currentBot` instead of scattering conditionals throughout mechanisms or op modes.
- Use `Direction.FORWARD` / `Direction.REVERSE` for drive motors in profiles.

## Important separation of concerns

Do not use `RobotConstants` as a catch-all for every subsystem on the robot.

- `RobotConstants` / `RobotConfigurations` are for drivetrain, localizer, follower, pathing profile data, and which `Mechanism` classes are enabled on a given robot.
- Mechanism-specific behavior or helper params such as intake power values or Limelight mounting values should stay with their mechanism/wrapper type rather than bloating the robot-profile object.

That separation keeps the robot profile focused on what varies per bot while reusable subsystem logic stays with the subsystem itself.

## Current fields to know

`RobotConstants` already owns:

- drive motor names and directions
- `IMU_HARDWAREMAP_NAME`, `IMU_LOGO_FACING_DIRECTION`, and `IMU_USB_FACING_DIRECTION`
- `PinpointConstants PINPOINT_LOCALIZER_CONSTANTS`
- max drivetrain power
- Pedro x/y velocity values
- path constraint tuning
- heading PIDF coefficients
- centripetal scaling
- enabled mechanism classes via the `Mechanisms` `Set`, `configureDriveMechanisms()`, and `addMechanism(...)`
- follower-drive compatibility through `USE_FOLLOWER_DRIVE`

## Rules

- Reuse the existing fluent builder methods on `RobotConstants` instead of introducing parallel drivetrain configuration objects.
- Keep all four drive motor names and directions together when changing a drivetrain profile.
- Configure each profile's IMU name and REV Hub logo/USB directions with `imuHardwareMapName(...)`, `imuLogoFacingDirection(...)`, and `imuUsbFacingDirection(...)`.
- Create IMUs through `Constants.createIMU(hardwareMap)`; it initializes orientation but deliberately does not reset yaw.
- Keep Pinpoint-related updates bundled through `pinpointConstants(...)`.
- Keep drive motor directions on the shared `Direction` enum so `Drivetrain` and `RobotConstants.createFollower(...)` stay aligned.
- Use `configureDriveMechanisms()` when you want the normal drive stack (`MecanumDrive` + `Drivetrain`) enabled by default.
- When adding a new subsystem that should participate in the profile system, register it with `.addMechanism(...)` and give it the constructor shape that `RobotConstants.createMechanism(...)` expects.
- Do not create ad hoc `FollowerBuilder` calls elsewhere when `Constants.createFollower(...)` or `RobotConstants.createFollower(...)` can be reused.
- Do not move mechanism-specific params into `RobotConstants` unless they truly vary by robot profile.
- Treat the current values as placeholders unless they were explicitly measured on the robot.

## Example workflow

1. Add or update a preset in `RobotConfigurations`.
2. Point `Constants.currentBot` at that preset.
3. Confirm `RobotConstants.createFollower(...)` still matches the chosen localizer and drivetrain style.
4. If the preset should support the normal drive stack, include `.configureDriveMechanisms()`.
5. If a new mechanism is involved, register it with `.addMechanism(...)`, confirm its constructor matches `(HardwareMap, RobotConstants)`, and optionally expose a `Constants.create...(...)` helper.
6. Validate any op mode or mechanism that depends on the follower.

## Common pitfalls in this repo

- `Constants` is a convenience surface, but the actual uniform mechanism factory logic now lives in `RobotConstants.createMechanism(...)`.
- `MecanumDrive` depends on the same active robot profile, so follower changes affect teleop and autonomous.
- `RobotConstants.createFollower(...)` currently assumes Pinpoint localization and the GoBILDA 4-bar pod resolution.
- `RobotConstants.createMechanism(...)` throws if the requested class is not present in the active profile’s `Mechanisms` list.
- `Constants.assertHasMechanism(...)` now fails earlier for convenience factory calls, so profile registration issues can show up before reflection-based construction.
- The presence of `Constants.createIntake(...)`, `createLimelight(...)`, `createRGB(...)`, or `createWebcam(...)` does not mean the active profile enables those classes. `PreseasonDrivetrain()` currently does not add `Intake`, `Limelight`, `RGBIndicator`, `LogitechWebcam`, or `Slides1`.
- `tests/MecanumTeleOp.java` now reads drivetrain motor names and default directions from `Constants.currentBot`, so drivetrain-profile renames should keep that test aligned automatically.
- `tests/MecanumTeleOp.java` and `tests/FieldCentricMecanumTeleOp.java` create their IMU through `Constants.createIMU(hardwareMap)`, so profile orientation changes apply to both tests.
- Any doc or patch that still talks about `starterBot()` or says the active preset is `OldDECODERobotTesting()` is stale; `Constants.currentBot` currently selects `PreseasonDrivetrain()`.
