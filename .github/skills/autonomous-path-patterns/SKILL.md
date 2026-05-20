---
name: autonomous-path-patterns
description: Use this skill when asked to add or refactor autonomous geometry, `Path`/`PathChain` builders, or state-machine behavior in TeamCode autonomous op modes.
---

# Autonomous Path Patterns

Use this skill when autonomous work is mainly about path definitions and sequencing.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`

## Two autonomous styles in this repo

1. Ivy-driven sequencing, as in `IvyAuton`, where a variant supplies `IvyAuton_Paths` and the shared base schedules Pedro commands.
2. Manual iterative state machines, as in `PreseasonWebcamAuto`, where `loop()` checks follower state and advances enum-based stages.

## Path-building conventions

- Use `PathChain` for Ivy autonomous builders, even when there is only one `Path`.
- Use `BezierLine` or `BezierCurve` for geometry depending on whether the motion is straight or curved.
- Keep start pose and path geometry near each other.
- Set heading interpolation explicitly rather than relying on implicit defaults.

## Repo examples

- `Test_IvyAuton.Paths` returns a `PathChain` built from one `Path`.
- `PreseasonWebcamAuto` uses nested static pose/path helpers and a `State` enum to coordinate release, intake, and park behavior.
- `PedroPathingTeleOpTest` uses `follower.pathBuilder()` lazily to create a path from the current pose.

## Mechanism creation rule for autonomous work

If autonomous needs a new subsystem or vision wrapper, do not construct it ad hoc inside the path/state code.

Instead:

1. add or update the mechanism/wrapper params type
2. expose the shared editable params instance in `Constants`
3. add `Constants.create...(...)`
4. instantiate through `Constants` during op mode initialization

That keeps autonomous and teleop using the same subsystem construction path and lets Dashboard/panels tune params centrally.

## Rules

- Prefer path helpers or nested path classes over dumping raw coordinates directly into op mode control flow.
- Keep transition conditions tied to follower state like `isBusy()` or timing, not arbitrary delays alone.
- If a path needs a specific heading profile, encode it in the path builder next to the geometry.
- If the autonomous is variant-specific, put geometry in the variant class and keep shared orchestration in the base.
- Create mechanisms through `Constants`, not direct constructors sprinkled through state-machine branches.

## Things to watch

- `PreseasonWebcamAuto.Poses.generateIntakePath()` is still a stub, so do not assume every sample path is complete.
- Follower-localization choice matters if the camera pipeline is needed for vision tasks.
- Preserve pose handoff if the autonomous is expected to feed teleop state.

