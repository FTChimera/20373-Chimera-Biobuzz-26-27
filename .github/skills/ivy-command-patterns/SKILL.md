---
name: ivy-command-patterns
description: Use this skill when asked to compose Pedro Ivy commands, schedule robot behavior, or convert between `Runnable` and Ivy `Command` objects in TeamCode.
---

# Ivy Command Patterns

This skill covers the repo’s Pedro Ivy usage.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/kotlinTest/KtExampleIvyAuto.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyWithSubsystems.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/CommandConverter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Ivy.java`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js`

## Current usage patterns

1. `Scheduler.reset()` is called at op mode startup or init.
2. `Scheduler.schedule(...)` is used to install long-running teleop and autonomous behavior.
3. `Scheduler.execute()` is called every loop in iterative Ivy flows.
4. Mechanisms expose small Ivy-friendly helpers like `Intake.fullPower()` or `MecanumDrive.periodicCommand()`.
5. `CommandConverter.toCommand(...)` wraps `Runnable` actions as instant commands.

## Common repo examples

- `IvyTeleOp` schedules Limelight updates, drive periodic updates, teleop drive, intake power, and heading reset behavior together.
- `IvyAuton.start()` composes a sequential routine with intake commands, Pedro follow commands, waits, and an op-mode stop request.
- `learning/ExampleIvyAuto` is a scratch iterative example that schedules `Commands.infinite(follower::update)` alongside a `Groups.sequential(...)` chain of `PedroCommands.follow(...)` calls; `learning/kotlinTest/KtExampleIvyAuto.kt` mirrors that pattern in Kotlin.
- `MecanumDrive.teleOp(...)` returns a command that starts teleop drive once and then runs continuous drive updates.
- `Slides1` exists as a mechanism shell but does not yet expose Ivy command helpers or active-profile registration, so new command work around slides may need both subsystem and profile follow-up.

## Input-wrapper Ivy convention

If an input wrapper exposes Ivy support, prefer one command that updates the full wrapper state.

- `DualGamepad.updateCommand()` should be preferred over building separate commands that update `gamepad1` and `gamepad2` independently.
- Keep input-update ownership in one place so button edges, touchpad state, and stick deltas stay synchronized across both pads.
- If a wrapper already has `update()` plus `updateCommand()`, reuse that pair instead of rebuilding the same `Commands.infinite(...)` inline everywhere.

## How Ivy work should construct subsystems

Ivy commands should operate on mechanisms that were created through `Constants` factories.

- create the mechanism once through `Constants.create...(...)`
- schedule commands against that mechanism instance
- if a new mechanism is needed, make it fit the `Mechanism` + `RobotConstants.createMechanism(...)` pattern first

Do not hide subsystem construction or configuration policy inside a custom command class if it really belongs in the mechanism or robot-profile layer.

## Rules

- Reset the scheduler before scheduling new work in an op mode.
- Keep one clear execution loop for scheduled commands.
- Prefer mechanism-level command helpers for reusable actions instead of rebuilding the same command inline in every op mode.
- Use `Commands.infinite(...)`, `Groups.sequential(...)`, and related Pedro Ivy primitives before inventing a custom loop.
- Use `CommandConverter.toCommand(...)` for simple instant actions; use full command composition for ongoing behaviors.
- If a task adds a new commandable mechanism, also add its profile registration and `Constants.create...(...)` helper when needed.

## When to use `Ivy.run(...)`

`pedroPathing/Ivy.java` is a synchronous helper that runs one command to completion in a blocking style. Use it carefully. It is not the main pattern for standard FTC loop-driven op modes.

## Generated Ivy skeletons

- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js` now emits the same basic Ivy helper shape that `ExampleIvyAuto` demonstrates: `reset()` in init/stop, `schedule(...)` in `start()`, and `Scheduler.execute()` in `loop()`.
- Generated code is a starting point only. Before keeping it, align package names, annotations, `Constants` imports, and any shared-base-class decisions with the real destination op mode.

## Scratch-example status

- `learning/ExampleIvyWithSubsystems.java` is a teaching scaffold, not a profile-managed subsystem example: its unfinished `autoAlign()` step is commented out, while intake uses a direct `DcMotor` resolved from `Intake.INTAKE_PARAMS`.
- Keep new command work on the normal `Constants.create...(...)` mechanism path rather than copying its direct intake construction.

## Pitfalls

- Do not forget `Scheduler.execute()` in iterative command-driven op modes.
- Avoid scheduling duplicate periodic loops that both call into the same mechanism every frame.
- Avoid mixing busy loops with Ivy scheduling when Pedro already provides a command form.
- Avoid creating or wiring profile-managed subsystems ad hoc inside commands; initialize them through the normal factory path first.
- If a test or proof-of-concept command intentionally bypasses a wrapper, still keep hardware naming centralized through the active profile or the owning params class.
