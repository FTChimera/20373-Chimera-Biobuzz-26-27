---
name: ivy-command-patterns
description: Use this skill for Pedro Ivy command composition and scheduler ownership.
---

# Ivy Command Patterns

Use Ivy commands with a single, visible scheduler lifecycle.

## Current autonomous scheduler

- `autos/Garden8` is the scheduler-backed competition autonomous.
- It calls `Scheduler.reset()` in `init()`.
- It builds its route through `PlanBuilder`.
- `PlanBuilder.schedule()` returns the plan's Ivy `Command`.
- `Garden8.loop()` calls `Scheduler.execute()`.
- `Garden8.stop()` resets the scheduler after stopping the robot.
- Keep this reset/build/execute/reset ownership intact.

## Robot command builder

- `Robot.build()` returns `RobotIvyBuilder`.
- Use it for existing robot lifecycle, follower, intake, drive, vision, and relocalization commands.
- A builder command operates only on mechanisms configured in that robot.
- Check profile eligibility before scheduling a command that requires a mechanism.
- Prefer builder commands over duplicating mechanism lifecycle calls.

## Planner commands

- `PlanBuilder` accumulates a `PathPlannerList`.
- `build()` returns the list when further inspection is needed.
- `schedule()` turns that list into an Ivy command.
- Use `startGardenPose(robot)` for implemented garden plans.
- Respect its tracked planner position.
- Invalid position transitions deliberately throw `IllegalStateException`.
- Compose custom `PathChain` steps with `followPath(...)`.

## Command lifetime

- Every command needs a clear start condition, owner, and completion behavior.
- Do not repeatedly schedule the same command in every loop.
- Keep command scheduling outside the loop unless an event edge intentionally creates a new command.
- Use an infinite command only for a service that must run until scheduler reset or cancellation.
- Ensure competing drive commands do not own the drivetrain together.

## Input service commands

- `DualGamepad.updateCommand()` creates an infinite update command.
- It updates both controllers.
- Schedule it only when normal loop code is not already calling `update()`.
- Never use both update paths for the same controllers in one cycle.
- `GpBindings` remains a view of the latest updated state.

## Robot periodic service

- `Robot.schedule(stateSupplier)` schedules infinite aggregate periodic dispatch.
- The supplier must return a non-null, mode-compatible `State`.
- Do not also call `Robot.periodic(state)` manually when that service is scheduled.
- Every mechanism receives `periodic(State)` through the aggregate.
- Mechanisms expose `stop()` for final hardware shutdown.

## Autonomous shutdown

- The Garden route applies match-time behavior outside the plan command.
- On timeout it requests robot stop without immediate pose saving.
- It saves values after velocity is low or a short delay expires.
- Its final `stop()` avoids stopping twice.
- `Robot.saveEndVals()` persists autonomous pose and alliance to Blackboard when state exists.

## Pedro dependency behavior

- Verify Ivy APIs against the pinned Gradle dependency before relying on memory.
- Inspect the cached dependency source or classes for scheduler semantics.
- Do not document command cancellation behavior without source confirmation.
- Keep Pedro follower updates and Ivy command execution coordinated.

## Review checklist

- Is the scheduler reset before a new autonomous plan?
- Is the command scheduled once?
- Is `Scheduler.execute()` called once per iterative loop?
- Is a drivetrain command exclusive?
- Is the robot stopped on every exit path?
- Is state dispatch owned by either the loop or a service command, not both?
- Does final shutdown clear scheduled work?

## Testing scheduler work

- Test a command's normal completion and cancellation behavior.
- Test the autonomous timeout path while a plan is still active.
- Test scheduler reset after an early stop.
- Confirm no command writes drive output after aggregate robot shutdown.
- Verify command dependencies against the pinned Ivy library version.
- Use telemetry to identify the currently active route stage on hardware.
- Keep experimental scheduler flows outside competition entries until verified.
- Compile and then exercise FTC lifecycle behavior on the robot.

## Change boundaries

- Keep scheduling policy in the entry or route owner.
- Keep command mechanics in `RobotIvyBuilder` or a planner module.
- Keep hardware shutdown in the aggregate robot lifecycle.
- Review cancellation paths when moving a command across these boundaries.
