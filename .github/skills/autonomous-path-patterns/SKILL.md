---
name: autonomous-path-patterns
description: Use this skill for autonomous geometry, Pedro paths, and planner composition.
---

# Autonomous Path Patterns

Use this guide when editing autonomous path geometry or planner-backed routines.

## Current entry point

- `autos/Garden8.java` is the concrete competition autonomous flow.
- Its nested `redalliance` and `bluealliance` classes are the annotated choices.
- Each class stores its alliance in `Blackboard` before the parent lifecycle runs.
- `Garden8` is an iterative FTC `OpMode`.
- `init()` resets Ivy's `Scheduler`, creates the aggregate `Robot`, and builds a plan.
- `start()` starts the robot.
- `loop()` executes scheduled commands and applies the end-of-match safety logic.
- `stop()` stops the robot if needed and resets the scheduler.

## Planner composition

- Start garden routines with `PlanBuilder.startGardenPose(robot)`.
- Start a deliberately empty plan with `PlanBuilder.nullPlan(robot)`.
- `PlanBuilder` owns a `PathPlannerList`.
- It tracks its logical position so invalid routine transitions throw early.
- `shootPreloads()` is supported from the garden start.
- `intakeGarden()` is supported after the garden shooting position.
- The vision variants move the tracked position to the garden vision-search state.
- Loading-side modules remain explicitly incomplete; do not document them as available.
- `build()` returns the planner list.
- `schedule()` returns the Ivy `Command` from that list.

## Custom path steps

- Use `followPath(PathChain)` to add a prebuilt chain.
- Use `linearPathTo(Pose)` for a line from the previous planner endpoint.
- Use `waitMs(double)` for a timed planner step.
- `intakeWVision(...)` is deprecated; use an explicit, reviewed path module instead.
- Preserve the planner's position invariant when adding new fluent methods.
- A path module should own geometry close to the route it represents.
- Existing modules live in `library/planners/paths/`.

## Shared route data

- `Common` is the shared autonomous-values holder.
- It contains timing values, tunable path values, and alliance-aware pose groups.
- Call `Common.init(AllianceColor)` before consuming its alliance pose values.
- `Robot` does this during construction using its selected alliance.
- `AlliancePoses` supplies the active alliance pose from paired red/blue values.
- The default `Common` poses are placeholders until the route is calibrated.
- Do not present zero-value poses as field-ready coordinates.

## Vision intake paths

- `IntakeBallPath` converts detector results into a smooth `PathChain`.
- Use the overload receiving `Follower`, robot `Pose`, `Vision`, ball count, and detections.
- It rejects invalid ball counts and empty detection lists.
- It filters confidence, target area, range, and lateral offset.
- It deduplicates nearby candidates and chooses a scored forward sequence.
- It builds field poses in the configured FTC coordinate system.
- It uses the Java `MathUtils` geometry helpers for angles, distances, and offsets.
- The no-argument overload returns an empty `PathChain` scaffold.
- Do not treat that no-argument overload as camera-driven behavior.

## Robot lifecycle in autonomous

- Create the robot through `Constants.createRobot(...)`.
- Match its mode to the `State` supplied to `Robot.periodic(State)`.
- `Robot` dispatches every configured mechanism's periodic update.
- Each mechanism implements `periodic(State)` and `stop()`.
- `Robot.stop(boolean)` stops mechanisms and optionally saves end state.
- `Garden8` delays pose saving after timeout until velocity is low or a timer expires.
- `Robot.saveEndVals()` writes autonomous pose and alliance only after a periodic state exists.
- Teleop reads the saved Blackboard pose as its initial pose.

## Pedro and Ivy

- Use `Scheduler.schedule(...)` only for commands with a defined owner and end condition.
- Run `Scheduler.execute()` once in the iterative autonomous loop.
- Reset the scheduler in both autonomous initialization and final shutdown.
- Use the robot's `build()` API when a standard mechanism or follower command exists.
- Keep direct follower updates and scheduled path commands coordinated.
- Check Pedro dependency APIs in Gradle caches before documenting unfamiliar behavior.

## Review checklist

- Verify coordinates and headings against the active alliance and coordinate system.
- Verify every planner transition has a legal preceding position.
- Verify required profile mechanisms are registered before calling `require...()`.
- Keep fallback and detector failure paths safe and explicit.
- Verify timeout, stop, and pose-handoff behavior on the robot.
- Run the focused validation described in `build-and-validation`.

## Calibration notes

- Tune routes with the same profile and localization setup used in matches.
- Measure poses from a defined field origin before entering them in a module.
- Validate a route in both alliances after mirroring or pairing poses.
- Keep headings in the unit expected by Pedro `Pose`.
- Test empty detector output before enabling vision-driven collection.
- Record intentional path assumptions in the owning path module.
- Rebuild after changing Java source or annotation-backed entry points.
- Use Driver Station testing before relying on a desktop-only result.
