---
name: pedro-pathing-and-ivy
description: Use this skill when asked to add or tune Pedro Pathing followers, paths, localizers, or Ivy scheduler commands in this FTC repository.
---

# Pedro Pathing and Ivy in This Repository

This repository uses Pedro Pathing for localization, teleop drive support, and autonomous path following, plus Pedro Ivy for command scheduling.

## Key files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Mechanism.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`

## How this repo is wired

1. Follower creation is exposed through `Constants.createFollower(...)`.
2. The actual Pedro drivetrain, Pinpoint, and path-constraint values live under `library/robot/RobotConstants` and `RobotConfigurations`.
3. `MecanumDrive` wraps a Pedro `Follower` plus the team `Drivetrain`.
4. Reusable mechanisms and wrappers now share a `Mechanism` base class and are instantiated uniformly through `RobotConstants.createMechanism(...)`, usually via `Constants.create...(...)` convenience helpers.
5. Ivy-based autonomous code usually defines an `IvyAuton_Paths` implementation in a variant class and passes it into `IvyAuton`.
6. Drive motor directions inside the robot profile now use the shared `library.hardware.actuators.Direction` enum before converting to the FTC SDK types Pedro expects.
7. Custom path geometry can now use `library/math/Polynomial` plus `pedroPathing/custom/EquationCurve` when Bezier primitives are not the best fit.
8. `RobotConstants.createFollower(...)` currently builds a Pinpoint-based follower by default; `LimelightPinpointLocalizer` exists separately and is not wired into the default factory.
9. `OpModeHelper.kt` is a reusable utility if a Pedro-related op mode wants manual bulk-cache clearing or loop timing without re-implementing that plumbing.

## Use these patterns

1. Add follower or localizer changes in `Constants` and `library/robot/...`, not scattered across op modes.
2. Put autonomous geometry in a variant-local `Paths` class that extends `IvyAuton.IvyAuton_Paths`.
3. Return a `PathChain` from autonomous path builders, even for a single `Path`.
4. Use `BezierLine`, `BezierCurve`, or `EquationCurve` based on the geometry you actually need.
5. For teleop with Ivy, schedule subsystem periodic commands and long-running control commands through the scheduler.
6. For non-Ivy pathing code, call `follower.update()` every loop.

## Mechanism construction around Pedro code

If a Pedro-related task adds a new mechanism or helper that owns robot behavior:

- make it extend `Mechanism` if it participates in the normal robot subsystem flow
- give it the `(HardwareMap, RobotConstants)` constructor that `RobotConstants.createMechanism(...)` expects
- register it with `.addMechanism(...)` in each relevant robot profile
- add a `Constants.create...(...)` helper when the subsystem should be easy to access from op modes
- do not scatter direct constructor calls across op modes

This applies around Pedro code too. `RobotConstants` remains the place for drivetrain/localizer/follower profile data, while robot-behavior classes use the uniform `Mechanism` construction path.

## `EquationCurve` and `Polynomial`

Use `EquationCurve` when the path is easier to describe as `y = f(x)` than as control points.

- `EquationCurve` extends Pedro `CustomCurve`.
- It takes `startPose`, `endPose`, and a `Polynomial`.
- The constructor currently validates endpoints with `MathFunctions.roughlyEquals(..., 1e-3)`, so the polynomial still needs to match the endpoint poses closely.
- `t` is mapped linearly across the start and end `x` positions.
- Derivatives are computed from `Polynomial.derivative()`, so math changes in `Polynomial` directly affect curve-following behavior.
- `Polynomial.toString()` is safe to use for debugging/telemetry-style inspection of decimal coefficients; it no longer relies on fragile fixed substring lengths.

## Ivy command construction

Use one of these three command patterns when adding new Ivy behavior:

1. Implement `Command` directly when the command has custom lifecycle behavior and should own all of `requirements()`, `priority()`, `start()`, `execute()`, `done()`, and `end(...)`.
2. Use `new CommandBuilder()` for most inline or one-off commands.
3. Extend `CommandBuilder` when you want a reusable base command with builder-style configuration.

`CommandBuilder` already implements `Command`, so the normal pattern is to configure it with:

- `setStart(...)`
- `setExecute(...)`
- `setDone(...)`
- `setEnd(...)`
- `requiring(...)`
- `setPriority(...)`
- `setInterruptedBehavior(...)`
- `setBlockedBehavior(...)`
- `setConflictBehavior(...)`

Prefer `new CommandBuilder()` for lightweight commands that are more complex than a simple `Runnable`, and prefer implementing `Command` or extending `CommandBuilder` for reusable command types.

Examples:

```java
Command command = new CommandBuilder()
        .setStart(() -> intake.setPower(1.0))
        .setDone(() -> false)
        .setExecute(drive::periodic)
        .requiring(intake, drive);
```

```java
public final class IntakeCommand extends CommandBuilder {
    public IntakeCommand(Intake intake, double power) {
        setStart(() -> intake.setPower(power));
        setDone(() -> true);
        requiring(intake);
    }
}
```

## Important repo-specific constraints

- `RobotConfigurations.OldDECODERobotTesting()` still contains placeholder tuning values for follower, drivetrain, and pinpoint offsets; do not assume those numbers are competition-ready.
- `LimelightPinpointLocalizer` is deprecated and should be treated as an opt-in experiment, not the default path.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept validation op mode that uses the follower plus raw Limelight color results; it is not the shared match autonomous architecture.
- That proof-of-concept Limelight test now still sources its camera hardware name from `Limelight.LIMELIGHT_PARAMS`, even though it bypasses the wrapper for behavior.
- `tests/PreseasonWebcamAuto.java` also relies on generic `PIDFCoefficients<Pose>` scalar broadcasting, so utility changes in `library/control/` can affect that test flow directly.
- If a new Pedro-adjacent mechanism is created, it should still follow the `Mechanism` + `RobotConstants.createMechanism(...)` pattern.

## Rules

- Reuse the existing `Constants.createFollower(...)` → `RobotConstants.createFollower(...)` flow instead of constructing ad hoc followers elsewhere.
- Keep localization decisions explicit: choose Pinpoint-only or Limelight-plus-Pinpoint based on the active vision pipeline.
- Do not add duplicate drive update loops; there should be one consistent source of `follower.update()`.
- Prefer Pedro/Ivy commands for path following in Ivy op modes instead of custom busy loops.
- Keep non-drive subsystem behavior out of `RobotConstants`; reserve that class for profile data and mechanism registration.
- Reach for `EquationCurve` only when its polynomial-through-endpoints model matches the path you want better than Bezier geometry.

## Example workflow

1. Define or update path geometry in an `IvyAuton_Paths` subclass.
2. Keep start pose and `PathChain` together.
3. In `IvyAuton.start()`, compose behavior with `Groups.sequential(...)`.
4. If the routine adds a new subsystem, register it in the active robot profile and expose it through `Constants.create...(...)` before wiring it into the command sequence.
5. Stop the op mode through `CommandConverter.toCommand(this::requestOpModeStop)` when the sequence finishes.
