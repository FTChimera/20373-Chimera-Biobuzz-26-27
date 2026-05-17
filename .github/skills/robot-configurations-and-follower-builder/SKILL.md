---
name: robot-configurations-and-follower-builder
description: Use this skill when asked to edit robot profiles, drivetrain naming/directions, Pinpoint offsets, or Pedro follower construction in `Constants`, `RobotConfigurations`, or `RobotConstants`.
---

# Robot Configurations and Follower Builder

Use this skill for robot-profile work centered around:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`

## How this repo is wired

1. `Constants.currentBot` selects the active robot profile.
2. `RobotConfigurations` defines named robot presets such as `starterBot()`.
3. `RobotConstants` is a builder-style container for drive names, directions, Pinpoint offsets, follower tuning, and path constraints.
4. `Constants.createFollower(hardwareMap)` delegates follower creation to the active `RobotConstants` instance.
5. `RobotConstants.createFollower(...)` builds the Pedro follower through `FollowerBuilder` using `FollowerConstants`, `MecanumConstants`, `PinpointConstants`, and `PathConstraints`.

## What belongs where

- Put global non-drive constants like intake or Limelight names in `Constants`.
- Put drive motor names, drive directions, Pinpoint encoder directions, and follower/pathing tuning in `RobotConstants`.
- Put concrete robot presets in `RobotConfigurations`.
- Prefer selecting a profile through `Constants.currentBot` instead of scattering conditionals throughout mechanisms or op modes.

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

## Rules

- Reuse the existing fluent builder methods on `RobotConstants` instead of introducing parallel configuration objects.
- Keep all four drive motor names and directions together when changing a drivetrain profile.
- Keep Pinpoint geometry and encoder-direction updates in the same robot profile change.
- Do not create ad hoc `FollowerBuilder` calls elsewhere when `Constants.createFollower(...)` or `RobotConstants.createFollower(...)` can be reused.
- Treat the current values as placeholders unless they were explicitly measured on the robot.

## Example workflow

1. Add or update a preset in `RobotConfigurations`.
2. Point `Constants.currentBot` at that preset.
3. Confirm `RobotConstants.createFollower(...)` still matches the chosen localizer and drivetrain style.
4. Validate any op mode or mechanism that depends on the follower.

## Common pitfalls in this repo

- `Constants` is intentionally thin; much of the drive and follower detail now lives under `library/robot/`.
- `MecanumDrive` depends on `Constants.createFollower(...)`, so follower changes affect teleop and autonomous.
- `RobotConstants.createFollower(...)` currently assumes Pinpoint localization and the GoBILDA 4-bar pod resolution.

