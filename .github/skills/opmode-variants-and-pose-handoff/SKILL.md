---
name: opmode-variants-and-pose-handoff
description: Use this skill when asked to add annotated variants, share logic across op modes, or preserve robot pose and shared driver state through `configs.Blackboard` between autonomous and teleop.
---

# OpMode Variants and Pose Handoff

Use this skill when working on the repo’s shared-base-class plus thin-wrapper pattern.

## Upstream `~/.gradle` lookup

If a variant or shared base class depends on external op-mode infrastructure,
inspect `~/.gradle/caches/modules-2/files-2.1/` with the versions pinned in
`build.dependencies.gradle`.

- `org.firstinspires.ftc/RobotCore` and `org.firstinspires.ftc/Hardware` for
  SDK op-mode and hardware behavior
- `com.pedropathing/ivy` for Ivy-based shared autonomous/teleop orchestration
- `com.pedropathing/telemetry` when `pedroPathing/Tuning.java` selection or
  registration is part of the issue

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/Blackboard.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ConfigOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Blue_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Red_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`

## Design intent

1. Shared robot behavior goes in a reusable base op mode.
2. Driver-station-visible entries stay small and hold the FTC annotation.
3. Alliance-specific differences should usually be expressed in wrappers or parameters.
4. Cross-opmode pose transfer and shared driver preferences go through `configs.Blackboard`.

## Current patterns in this repo

- `TeleOp` is an abstract `LinearOpMode` that accepts an `AllianceColor`, but the current base stores that value without branching on it yet.
- `Blue_TeleOp` and `Red_TeleOp` are the current concrete wrappers; each reads `Blackboard.Caption.POSE` first and only falls back to the alliance default constant when the stored pose is still `0,0,0`.
- `IvyTeleOp` is currently a standalone annotated teleop, so not every teleop in the repo uses the thin-wrapper pattern.
- `IvyAuton` is an abstract `OpMode` that accepts an `AllianceColor` plus an `IvyAuton_Paths` implementation, but the current base also stores rather than branches on the alliance value.
- `Test_IvyAuton` provides a nested `Paths` implementation and currently passes `AllianceColor.RED` to the shared base.
- `IvyAuton.stop()` calls `Robot.stop()` before resetting the Ivy scheduler.
  In autonomous mode, Robot stores a velocity-compensated end pose plus the
  state alliance in `Blackboard`.
- `ConfigOpMode` is the one iterative pre-start menu for `Blackboard` mutations. Its D-pad menus set `POSE`, `DRIVER`, or `ALLIANCE`, or reset every entry before the op mode is started.
- `OpModeHelper.kt` is available as shared lifecycle plumbing if a future base op mode needs loop timing or manual bulk-cache handling, but it is not itself an annotated op mode base.

## Mechanism construction rule for shared bases

Shared base op modes should initialize mechanisms through `Constants.create...(...)` so every variant uses the same profile-aware factories.

Variant wrappers inherit whatever profile and aggregate assumptions their base
class makes. For example, `TeleOp.java` creates `RobotMode.TELEOP` Robot and
State instances, then calls `requireDrive()` and optional `get(...)` lookups
for `Intake`; it requires `Vision`. The selected profile must still register
any class that the base treats as required.

If a new subsystem is introduced for a base op mode:

1. make it follow the repo’s `Mechanism` + `(HardwareMap, RobotConstants)` constructor pattern when it is a profile-managed subsystem
2. register it in the active robot profile with `.addMechanism(...)`
3. add `Constants.create...(...)` when the shared base should access it directly
4. initialize it in the shared base, not separately in every variant wrapper

## Rules

- Keep FTC annotations like `@TeleOp` and `@Autonomous` on concrete selectable classes, not on abstract bases.
- Prefer adding a new wrapper class in `opModes/variants/` over copy-pasting a full base class.
- If teleop needs autonomous pose handoff, read from and write to `Blackboard.Caption.POSE` intentionally.
- If you rely on pose handoff, preserve the current `0,0,0` guard in `Blue_TeleOp` / `Red_TeleOp` or replace it with another explicit "pose is valid" signal.
- Keep alliance-specific differences explicit through constructor parameters, paths, or start poses.
- Keep subsystem factories centralized through `Constants` / `RobotConstants.createMechanism(...)`, not redefined per variant.
- Use `ConfigOpMode` when the task is a one-shot configuration or reset flow rather than a continuously running teleop.

## Good use cases

- Add a `Blue_*` and `Red_*` pair with shared logic in one base class.
- Move duplicate driver code out of a variant and into `TeleOp`.
- Add a new autonomous by subclassing `IvyAuton` and supplying a repo-style `IvyAuton_Paths` object.

## Things to watch

- `Blackboard` is currently seeded with default `POSE`, `DRIVER`, and `ALLIANCE` entries, so consumers should understand the difference between “default value” and “meaningful restored value”.
- `LinearOpMode` and iterative `OpMode` bases behave differently; do not blindly move logic between them.
- Keep `waitForStart()` only in `LinearOpMode` flows.
