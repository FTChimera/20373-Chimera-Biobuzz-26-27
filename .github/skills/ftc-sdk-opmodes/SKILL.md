---
name: ftc-sdk-opmodes
description: Use this skill when asked to create or edit FTC SDK op modes, wire hardware through HardwareMap, add TeleOp or Autonomous classes, or debug FTC lifecycle issues in this TeamCode project.
---

# FTC SDK Op Modes for This Repository

Use this skill when working on FTC SDK code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

## Repository context

- The app uses FTC SDK `11.1.0`.
- `TeamCode/` is the team module; `FtcRobotController/` is the controller app plus SDK samples.
- Shared robot constants live in `Constants.java`.
- Pose handoff between op modes lives in `Blackboard.java`.
- Team op modes live under `opModes/`, with `variants/` holding annotated wrappers such as `Blue_TeleOp` and `Red_TeleOp`.

## Use these patterns

1. Put reusable op mode logic in a shared base class under `opModes/`.
2. Put driver-station-visible wrappers in `opModes/variants/` and annotate those with `@TeleOp` or `@Autonomous`.
3. Resolve hardware names from `Constants` instead of hardcoding strings in op modes.
4. Build mechanisms first, then wire them into the op mode.
5. If teleop needs the last autonomous pose, read and write `Blackboard.pose`.

## Current repo conventions

- Teleop drive is usually routed through `MecanumDrive`.
- Intake is wrapped by `mechanisms/Intake.java`.
- Limelight usage is wrapped by `library/hardware/webcam/Limelight.java`.
- Ivy-based op modes use `Scheduler.reset()`, `Scheduler.schedule(...)`, and `Scheduler.execute()`.
- Non-Ivy op modes call subsystem update methods directly inside `loop()` or the main `while (opModeIsActive())` block.

## Rules

- Keep FTC annotations on concrete, selectable classes only.
- Prefer adding alliance-specific wrappers over duplicating whole op modes.
- Do not bypass mechanism wrappers with raw `DcMotorEx` or `Limelight3A` access unless the wrapper is being improved.
- Keep hardware names and motor directions centralized in `Constants`.
- Preserve FTC lifecycle expectations: initialize hardware before using it, update telemetry in the loop, and stop or persist state in `stop()` when needed.

## Examples in this repo

- Shared teleop base: `opModes/TeleOp.java`
- Ivy teleop: `opModes/IvyTeleOp.java`
- Ivy autonomous base: `opModes/IvyAuton.java`
- Annotated wrappers: `opModes/variants/Blue_TeleOp.java`, `opModes/variants/Red_TeleOp.java`, `opModes/variants/Test_IvyAuton.java`

## Validation

- Build with Gradle before concluding changes are done.
- This project uses Android Gradle Plugin `8.7.0`, so builds require Java 11 or newer.
