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
2. `RobotConfigurations` currently defines named robot presets such as `OldDECODERobotTesting()`.
3. `RobotConstants` is a builder-style container for drive names, drive directions, Pinpoint offsets, follower tuning, path constraints, and the list of enabled `Mechanism` classes.
4. `Constants.createFollower(hardwareMap)` delegates follower creation to the active `RobotConstants` instance.
5. `RobotConstants.createFollower(...)` builds the Pedro follower through `FollowerBuilder` using `FollowerConstants`, `MecanumConstants`, `PinpointConstants`, and `PathConstraints`.
6. Drive motor directions in `RobotConstants` use the shared `library.hardware.actuators.Direction` enum and convert to FTC SDK directions only when building drivetrain/follower objects.
7. `RobotConstants.createMechanism(...)` is the uniform creation path for registered mechanisms and wrappers that follow the `(HardwareMap, RobotConstants)` constructor pattern.

## What belongs where

- Put drive motor names, drive directions, Pinpoint encoder directions, follower/pathing tuning, and enabled mechanism classes in `RobotConstants`.
- Put concrete robot presets in `RobotConfigurations`.
- Put top-level convenience helpers such as `createIntake(...)` and `createLimelight(...)` in `Constants` when they make op-mode code cleaner.
- Keep mechanism-specific behavior and optional nested params types in the owning mechanism or wrapper class.
- Prefer selecting a profile through `Constants.currentBot` instead of scattering conditionals throughout mechanisms or op modes.
- Use `Direction.FORWARD` / `Direction.REVERSE` for drive motors in profiles; keep GoBILDA Pinpoint encoder directions on the SDK enum that Pedro expects.

## Important separation of concerns

Do not use `RobotConstants` as a catch-all for every subsystem on the robot.

- `RobotConstants` / `RobotConfigurations` are for drivetrain, localizer, follower, pathing profile data, and which `Mechanism` classes are enabled on a given robot.
- Mechanism-specific behavior or helper params such as intake power values or Limelight mounting values should stay with their mechanism/wrapper type rather than bloating the robot-profile object.

That separation keeps the robot profile focused on what varies per bot while reusable subsystem logic stays with the subsystem itself.

## Current fields to know

`RobotConstants` already owns:

- drive motor names and directions
- Pinpoint hardware name and encoder directions
- Pinpoint pod offsets in inches
- max drivetrain power
- Pedro x/y velocity values
- path constraint tuning
- predictive braking coefficients
- heading PIDF coefficients
- centripetal scaling
- enabled mechanism classes via `Mechanisms` and `addMechanism(...)`

## Rules

- Reuse the existing fluent builder methods on `RobotConstants` instead of introducing parallel drivetrain configuration objects.
- Keep all four drive motor names and directions together when changing a drivetrain profile.
- Keep Pinpoint geometry and encoder-direction updates in the same robot profile change.
- Keep drive motor directions on the shared `Direction` enum so `Drivetrain` and `RobotConstants.createFollower(...)` stay aligned.
- When adding a new subsystem that should participate in the profile system, register it with `.addMechanism(...)` and give it the constructor shape that `RobotConstants.createMechanism(...)` expects.
- Do not create ad hoc `FollowerBuilder` calls elsewhere when `Constants.createFollower(...)` or `RobotConstants.createFollower(...)` can be reused.
- Do not move mechanism-specific params into `RobotConstants` unless they truly vary by robot profile.
- Treat the current values as placeholders unless they were explicitly measured on the robot.

## Example workflow

1. Add or update a preset in `RobotConfigurations`.
2. Point `Constants.currentBot` at that preset.
3. Confirm `RobotConstants.createFollower(...)` still matches the chosen localizer and drivetrain style.
4. If a new mechanism is involved, register it with `.addMechanism(...)`, confirm its constructor matches `(HardwareMap, RobotConstants)`, and optionally expose a `Constants.create...(...)` helper.
5. Validate any op mode or mechanism that depends on the follower.

## Common pitfalls in this repo

- `Constants` is a convenience surface, but the actual uniform mechanism factory logic now lives in `RobotConstants.createMechanism(...)`.
- `MecanumDrive` depends on `Constants.createFollower(...)`, so follower changes affect teleop and autonomous.
- `RobotConstants.createFollower(...)` currently assumes Pinpoint localization and the GoBILDA 4-bar pod resolution.
- `RobotConstants.createMechanism(...)` throws if the requested class is not present in the active profile’s `Mechanisms` list.
- Any doc or patch that still talks about `starterBot()` is stale; the current attached preset is `OldDECODERobotTesting()`.

