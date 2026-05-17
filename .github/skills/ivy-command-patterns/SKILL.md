---
name: ivy-command-patterns
description: Use this skill when asked to compose Pedro Ivy commands, schedule robot behavior, or convert between `Runnable` and Ivy `Command` objects in TeamCode.
---

# Ivy Command Patterns

This skill covers the repo’s Pedro Ivy usage.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/CommandConverter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Ivy.java`

## Current usage patterns

1. `Scheduler.reset()` is called at op mode startup or init.
2. `Scheduler.schedule(...)` is used to install long-running teleop and autonomous behavior.
3. `Scheduler.execute()` is called every loop in iterative Ivy flows.
4. Mechanisms expose small Ivy-friendly helpers like `Intake.fullPower()` or `MecanumDrive.periodicCommand()`.
5. `CommandConverter.toCommand(...)` wraps `Runnable` actions as instant commands.

## Common repo examples

- `IvyTeleOp` schedules Limelight updates, drive periodic updates, teleop drive, intake power, and heading reset behavior together.
- `IvyAuton.start()` composes a sequential routine with intake commands, `PedroCommands.follow(...)`, waits, and an op-mode stop request.
- `MecanumDrive.teleOp(...)` returns a command that starts teleop drive once and then runs continuous drive updates.

## Rules

- Reset the scheduler before scheduling new work in an op mode.
- Keep one clear execution loop for scheduled commands.
- Prefer mechanism-level command helpers for reusable actions instead of rebuilding the same command inline in every op mode.
- Use `Commands.infinite(...)`, `Groups.sequential(...)`, and related Pedro Ivy primitives before inventing a custom loop.
- Use `CommandConverter.toCommand(...)` for simple instant actions; use full command composition for ongoing behaviors.

## When to use `Ivy.run(...)`

`pedroPathing/Ivy.java` is a synchronous helper that runs one command to completion in a blocking style. Use it carefully. It is not the main pattern for standard FTC loop-driven op modes.

## Pitfalls

- Do not forget `Scheduler.execute()` in iterative command-driven op modes.
- Avoid scheduling duplicate periodic loops that both call into the same mechanism every frame.
- Avoid mixing busy loops with Ivy scheduling when Pedro already provides a command form.

