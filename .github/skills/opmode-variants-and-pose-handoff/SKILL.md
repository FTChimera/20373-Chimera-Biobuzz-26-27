---
name: opmode-variants-and-pose-handoff
description: Use this skill when asked to add annotated variants, share logic across op modes, or preserve robot pose through `Blackboard.pose` between autonomous and teleop.
---

# OpMode Variants and Pose Handoff

Use this skill when working on the repo’s shared-base-class plus thin-wrapper pattern.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Blue_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Red_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Blackboard.java`

## Design intent

1. Shared robot behavior goes in a reusable base op mode.
2. Driver-station-visible entries stay small and hold the FTC annotation.
3. Alliance-specific differences should usually be expressed in wrappers or parameters.
4. Cross-opmode pose transfer goes through `Blackboard.pose`.

## Current patterns in this repo

- `TeleOp` is an abstract `LinearOpMode` that accepts an `AllianceColor`.
- `IvyAuton` is an abstract `OpMode` that accepts an `AllianceColor` plus an `IvyAuton_Paths` implementation.
- `Test_IvyAuton` provides a nested `Paths` implementation and passes it to the shared base.
- `IvyAuton.stop()` writes `follower.getPose()` into `Blackboard.pose`.

## Rules

- Keep FTC annotations like `@TeleOp` and `@Autonomous` on concrete selectable classes, not on abstract bases.
- Prefer adding a new wrapper class in `opModes/variants/` over copy-pasting a full base class.
- If teleop needs autonomous pose handoff, read from and write to `Blackboard.pose` intentionally.
- Keep alliance-specific differences explicit through constructor parameters, paths, or start poses.

## Good use cases

- Add a `Blue_*` and `Red_*` pair with shared logic in one base class.
- Move duplicate driver code out of a variant and into `TeleOp`.
- Add a new autonomous by subclassing `IvyAuton` and supplying a repo-style `IvyAuton_Paths` object.

## Things to watch

- `Blackboard.pose` starts as `null`, so code that consumes it must tolerate missing pose state.
- `LinearOpMode` and iterative `OpMode` bases behave differently; do not blindly move logic between them.
- Keep `waitForStart()` only in `LinearOpMode` flows.

