---
name: ftc-sdk-opmodes
description: Use this skill when asked to create or edit FTC SDK op modes, wire hardware through HardwareMap, add TeleOp or Autonomous classes, or debug FTC lifecycle issues in this TeamCode project.
---

# FTC SDK Op Modes for This Repository

Use this skill when working on FTC SDK code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

## Repository context

- The app uses FTC SDK `11.1.0`.
- `TeamCode/` is the team module; `FtcRobotController/` is the controller app plus SDK samples.
- Shared robot constants and mechanism factories live in `Constants.java`, with drive-profile and follower configuration under `library/robot/`.
- Pose handoff between op modes lives in `Blackboard.java`.
- Team op modes live under `opModes/`, with `variants/` holding annotated wrappers such as `Blue_TeleOp` and `Red_TeleOp`.

## Use these patterns

1. Put reusable op mode logic in a shared base class under `opModes/`.
2. Put driver-station-visible wrappers in `opModes/variants/` and annotate those with `@TeleOp` or `@Autonomous`.
3. Resolve subsystem construction through `Constants.create...(...)` instead of direct constructors in op modes.
4. Let `Constants.create...(...)` delegate to the active robot profile when the subsystem is profile-managed.
5. If teleop needs the last autonomous pose, read and write `Blackboard.pose`.

## Central mechanism-creation convention

When an op mode needs a subsystem:

- call `Constants.createIntake(hardwareMap)` instead of `new Intake(...)`
- call `Constants.createLimelight(hardwareMap)` instead of `new Limelight(...)`
- call `Constants.createDrivetrain(hardwareMap)` or higher-level drive helpers instead of rebuilding drive hardware inline

When adding a brand-new mechanism, add all three pieces together:

1. make it extend `Mechanism` if it should participate in the standard robot-subsystem flow
2. give it the `(HardwareMap, RobotConstants)` constructor expected by `RobotConstants.createMechanism(...)`
3. register it in `RobotConfigurations` with `.addMechanism(...)`
4. add a `Constants.create...(...)` helper if op modes should access it directly

The important repo contract is that profile-managed subsystems can be created uniformly from the active `RobotConstants` instance.

## Current repo conventions

- Teleop drive is usually routed through `MecanumDrive`.
- Intake is wrapped by `mechanisms/Intake.java` and created through `Constants`.
- Limelight usage is wrapped by `library/hardware/webcam/Limelight.java` and created through `Constants`.
- Ivy-based op modes use `Scheduler.reset()`, `Scheduler.schedule(...)`, and `Scheduler.execute()`.
- Non-Ivy op modes call subsystem update methods directly inside `loop()` or the main `while (opModeIsActive())` block.
- `tests/MecanumTeleOp.java` is a current iterative FTC test op mode for raw mecanum drive, IMU yaw reset, and manual direction checks.

## Rules

- Keep FTC annotations on concrete, selectable classes only.
- Prefer adding alliance-specific wrappers over duplicating whole op modes.
- Do not bypass mechanism wrappers with raw `DcMotorEx` or `Limelight3A` access unless the wrapper is being improved.
- Keep subsystem creation on the `Constants` / `RobotConstants.createMechanism(...)` path; keep drive/follower profile data in `library/robot/RobotConstants`.
- Preserve FTC lifecycle expectations: initialize hardware before using it, update telemetry in the loop, and stop or persist state in `stop()` when needed.

## Examples in this repo

- Shared teleop base: `opModes/TeleOp.java`
- Ivy teleop: `opModes/IvyTeleOp.java`
- Ivy autonomous base: `opModes/IvyAuton.java`
- Annotated wrappers: `opModes/variants/Blue_TeleOp.java`, `opModes/variants/Red_TeleOp.java`, `opModes/variants/Test_IvyAuton.java`
- Standalone validation op mode: `tests/MecanumTeleOp.java`

## Validation

- Build with Gradle before concluding changes are done.
- This project uses Android Gradle Plugin `8.7.0`, so builds require Java 11 or newer.
- `TeamCode/build.gradle` now also includes Lombok annotation processing and Sloth load / runtime wiring, so build troubleshooting should include those pieces.
