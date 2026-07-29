---
name: ivy-command-patterns
description: Use this skill when asked to compose Pedro Ivy commands, schedule robot behavior, or convert between `Runnable` and Ivy `Command` objects in TeamCode.
---

# Ivy Command Patterns

This skill covers the repo’s current Pedro Ivy usage patterns, not generic Ivy examples from elsewhere.

## Relevant files

- `build.dependencies.gradle`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/gamepad/DualGamepad.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Slides1.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/CommandConverter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Ivy.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/IvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyWithSubsystems.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/kotlinTest/KtExampleIvyAuto.kt`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js`

## Verified current patterns

1. The repo builds against `com.pedropathing:ivy:1.0.0`.
2. Iterative Ivy flows reset the scheduler first, schedule work, and call `Scheduler.execute()` every loop.
3. TeamCode currently composes commands with `Commands.instant(...)`, `Commands.infinite(...)`, `Commands.waitMs(...)`, `Groups.sequential(...)`, `Groups.loop(...)`, and `PedroCommands.follow(...)`.
4. Mechanisms expose small Ivy-friendly helpers such as `Intake.fullPower()`, `Intake.setPowerCommand(...)`, `MecanumDrive.periodicCommand()`, `MecanumDrive.teleOp(...)`, and `Limelight.updateCommand()`.
5. `CommandConverter.toCommand(...)` wraps a `Runnable` as an instant command, while `CommandConverter.toRunnable(...)` runs a `Command` synchronously through `Ivy.run(...)`.
6. There is no current TeamCode example that implements `Command` directly, extends `CommandBuilder`, or constructs `new CommandBuilder()`.

## Common repo examples

- `opModes/IvyTeleOp.java` schedules Limelight updates, drive periodic updates, teleop drive, intake power control, and heading-reset behavior together.
- `opModes/IvyAuton.java` schedules a `Groups.sequential(...)` routine with intake commands, `PedroCommands.follow(...)`, `Commands.waitMs(...)`, and `requestOpModeStop`.
- `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` schedule `Commands.infinite(follower::update)` alongside a sequential follow-path chain.
- `learning/ExampleIvyWithSubsystems.java` is a teaching scaffold that still resolves its intake motor directly from `hardwareMap`.
- `DualGamepad.updateCommand()` exists as a wrapper-wide input-update command, but no current TeamCode op mode uses it yet.
- `Slides1` currently only wraps construction and does not expose Ivy command helpers.

## Profile / mechanism caveat

Ivy commands should operate on mechanisms created through the repo’s factory path:

- create the mechanism through `Constants.create...(...)`
- keep the mechanism constructor shape `(HardwareMap, RobotConstants)`
- register it in the selected `RobotConstants` profile

The `OldDECODERobotTesting` and `PreseasonDrivetrain` profiles exist in `RobotConfigurations`. Only mechanisms added with `.addMechanism(...)` in the profile selected by `Constants.currentBot` can be instantiated through factory helpers. Verify registration before writing Ivy commands that depend on a subsystem.

## Input-wrapper convention

If an input wrapper already exposes a repo-local update command, prefer it over scattering duplicate update loops.

- `DualGamepad.update()` updates both wrapped pads.
- `DualGamepad.updateCommand()` returns `Commands.infinite(this::update)`.
- Keeping wrapper updates centralized is the repo’s only current DualGamepad/Ivy precedent.

## `Ivy.run(...)`

`pedroPathing/Ivy.java` is a blocking helper that calls `start()`, loops on `execute()` until `done()`, then ends with `EndCondition.NATURALLY`. In current TeamCode, it is only used indirectly by `CommandConverter.toRunnable(...)`; it is not the main pattern for FTC loop-driven op modes.

## Generated Ivy skeletons

- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js` emits the same general helper shape shown in `ExampleIvyAuto`: `reset()` in init/stop, `schedule(...)` in `start()`, and `Scheduler.execute()` in `loop()`.
- Generated code is only a starting point. Align package names, annotations, `Constants` usage, and profile assumptions with the real destination op mode before keeping it.

## Rules

- Reset the scheduler before scheduling new work in an op mode.
- Keep one clear execution loop for scheduled commands.
- Prefer mechanism-level command helpers for reusable actions instead of rebuilding the same command inline everywhere.
- Use the repo’s existing `Commands` / `Groups` / `PedroCommands` composition style before adding a new command abstraction.
- Use `CommandConverter.toCommand(...)` for simple instant actions.
- If a task adds a new commandable mechanism, also add its profile registration and `Constants.create...(...)` helper when needed.
- If you introduce `CommandBuilder` or direct `Command` implementations, validate them against the pinned Ivy dependency because TeamCode itself does not currently demonstrate those patterns.

## Pitfalls

- Do not forget `Scheduler.execute()` in iterative command-driven op modes.
- Avoid scheduling duplicate periodic loops that call into the same mechanism every frame.
- Avoid mixing busy loops with Ivy scheduling when Pedro already provides a command form.
- Avoid creating profile-managed subsystems ad hoc inside commands; initialize them through the normal factory path first.
- If a proof-of-concept command intentionally bypasses a wrapper, still keep hardware naming centralized through the active profile or the owning params class.
