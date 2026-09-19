---
name: pedro-pathing-and-ivy
description: Use this skill for Pedro followers, paths, planner modules, and Ivy scheduling in TeamCode.
---

# Pedro Pathing and Ivy

The current robot pathing flow is:

`Constants.createRobot(...)` -> `RobotDrive` -> `MecanumDrive.follower` ->
`PlanBuilder` / `PathPlannerList` -> Ivy `Command` scheduled via
`com.pedropathing.ivy.Scheduler`.

## `PlanBuilder`: a stateful route composer

`PlanBuilder` tracks a `Positions` enum position (`GARDEN_START`,
`LOADING_START`, `GARDEN_SHOOT`, `LOADING_SHOOT`, `NULL`,
`GARDEN_VISION_SEARCH`, `LOADING_VISION_SEARCH`) and a `PathPlannerList`.
Start it with the static factories `PlanBuilder.startGardenPose(robot)`
(seeds a `FromStartGardenPose` planner at index 0) or `PlanBuilder.nullPlan(robot)`
(seeds a `Utils.StartPose` at `Constants.RED_DEFAULT_START_POSE`). Route
transition methods each assert the current `position` and throw
`IllegalStateException` on an invalid transition:

- `shootPreloads()` — from `GARDEN_START` adds
  `FromStartGardenPose.ShootPreloads`, moves to `GARDEN_SHOOT`; the
  `LOADING_START` branch is a `// Todo implement module` stub that still
  advances state without adding a planner.
- `shootPreloadsToVision()` — from `GARDEN_START` adds
  `FromStartGardenPose.ShootPreloadsToVisionSearch`, moves to
  `GARDEN_VISION_SEARCH`.
- `intakeGarden()` — from `GARDEN_SHOOT` adds
  `FromShootGardenPose.ToIntakeGardenPollen`.
- `intakeGardenToVision()` — from `GARDEN_SHOOT` adds
  `FromShootGardenPose.ToIntakeGardenPollenToVision`, moves to
  `GARDEN_VISION_SEARCH`.

Utility methods `followPath(PathChain)`, `linearPathTo(Pose)`, `waitMs(double)`
wrap `Utils.CustomPath`/`Utils.LinearPath`/`Utils.Wait`.
`intakeWVision(int, PathChain)` is `@Deprecated`. `build()`/`planner()` both
return the underlying `PathPlannerList`; `schedule()` calls
`list.schedule()`.

## `PathPlanner` / `PathPlannerList` contract

`PathPlanner` is a small interface: `getEndPose()`, `run()` (returns a
`Command`), `setStartPose(Pose)`, `buildPaths()`, a default `fromPrev(prev)`
that calls `setStartPose(prev.getEndPose().copy())`, and a default
`asCommand()` that sequences `instant(this::buildPaths)` then `run()` — so
path geometry is always rebuilt immediately before the module runs.
`PathPlannerList extends ArrayList<PathPlanner>`; its `asCommand()` walks the
list, inserting an `instant(() -> pathPlanner.fromPrev(prevPlanner))` before
each module's own `asCommand()` (starting from a `Utils.NullPlanner` sentinel),
which is how each module picks up the previous module's end pose without every
module needing manual wiring. `schedule()` builds that combined command and
calls `command.schedule()` on it directly.

Route modules belong in `library/planners/paths/`. Only `FromStartGardenPose`
and `FromShootGardenPose` are implemented today (they back the `PlanBuilder`
transitions above, plus an unused `ToIntakeClosestFlower` stub with a `null`
`run()`/`getEndPose()`). `FromVisionGardenPose`, `FromStartLoadingPose`,
`FromShootLoadingPose`, and `FromVisionLoadingPose` are empty placeholder
classes with no `PathPlanner` implementations yet — do not assume a loading-
side or vision-anchor route module exists until one of these is filled in.
`autos/Garden8` is the current annotated planner consumer — the earlier
`IvyAuton` / path-provider pattern was deleted from source.

## Ivy command helpers

- `library/robot/RobotIvyBuilder` (`Robot.build()`) wraps common robot actions
  as `Command`s: `followPath(...)`, `turnTo(...)`, `hold(...)` (all via
  `PedroCommands` against `robot.requireDrive().mecanumDrive.follower`),
  `startIntake()`/`runIntake(...)`/`stopIntake()`, `setTeleOpDrive(...)`,
  vision pipeline commands, and `forceRelocalize()`.
- `pedroPathing/Pedro.java` provides `runIvy(Command)` (a blocking
  start/execute-loop/end helper for contexts without a scheduler loop),
  `ivyFollowPath(follower, poses...)` (builds and follows a `BezierCurve`
  path with global linear heading interpolation), and `linear(poses...)` (a
  plain `Path` with linear heading interpolation, used throughout
  `library/planners/paths` to build inter-pose segments).

## Path and command rules

- Obtain followers through the active profile
  (`RobotConstants.createFollower(...)`); do not make independent
  `FollowerBuilder` instances in op modes.
- Put geometry and heading interpolation in the route module that owns it
  (e.g. `FromStartGardenPose.ShootPreloads.buildPaths()`), not in `PlanBuilder`.
- Use `RobotIvyBuilder` for aggregate follower/intake/vision commands rather
  than reaching into `Robot` mechanisms directly from planner code.
- Schedule exactly one follower update path — `Robot.periodic(state)` already
  drives `RobotDrive.periodic(state)` -> `MecanumDrive.periodic()` ->
  `follower.update()` once per loop.
- Treat `Common` and its `AlliancePoses` fields (`StartPoses`, `IntakePoses`,
  `ShootPoses`, `intake`, `park`, `intakeTest`) as uncalibrated placeholder
  `Pose()` values until final field measurements are entered.

Normal FTC op modes use `Scheduler.execute()` in their own loop (see
`Garden8.loop()`) rather than `Pedro.runIvy(...)`, which is intended for
one-off blocking sequences outside the main scheduler loop.
