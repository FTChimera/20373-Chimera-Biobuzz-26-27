---
name: pedro-pathing-and-ivy
description: Use this skill when asked to add or tune Pedro Pathing followers, paths, localizers, or Ivy scheduler commands in this FTC repository.
---

# Pedro Pathing and Ivy in This Repository

This repo currently builds against `com.pedropathing:ftc:2.1.2`, `com.pedropathing:telemetry:1.0.0`, and `com.pedropathing:ivy:1.0.0` (`build.dependencies.gradle`).

## Key files

- `build.dependencies.gradle`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Equation.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Mechanism.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/CommandConverter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Ivy.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyWithSubsystems.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/kotlinTest/KtExampleIvyAuto.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.html`
- `java-autogen/Ftc-PedroPathing-PathBuilder-toJavaAuto.html`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js`

## Verified repo wiring

1. `Constants.createFollower(...)` delegates to `Constants.currentBot.createFollower(...)`. The `OldDECODERobotTesting` and `FirstRobot` profiles exist in `RobotConfigurations`; `Constants.currentBot` selects the profile that drives the follower.
2. `RobotConstants.createFollower(...)` builds the Pedro follower through `FollowerBuilder(...).pathConstraints(...).mecanumDrivetrain(...).pinpointLocalizer(...).build()`.
3. `MecanumDrive` extends `Mechanism`, owns both a Pedro `Follower` and the team `Drivetrain`, and exposes `teleOp(...)` Ivy helpers while its per-loop update now runs through `periodic(Bindings bindings)`.
4. The match-style iterative autonomous path pattern is `opModes/IvyAuton.java` plus a variant-local `IvyAuton_Paths` implementation such as `opModes/variants/Test_IvyAuton.java`.
5. `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` are standalone iterative `OpMode` examples that compose `Scheduler`, `Commands.infinite(...)`, `Groups.sequential(...)`, and `PedroCommands.follow(...)` directly.
6. `EquationCurve` adapts `Equation` into a Pedro `CustomCurve`, but no current TeamCode op mode instantiates it.
7. The standard robot-subsystem construction path is `Mechanism` + `(HardwareMap, RobotConstants)` + `RobotConstants.createMechanism(...)` + `Constants.create...(...)`.
8. Profile-managed factory helpers work only for classes registered through `.addMechanism(...)` in the profile selected by `Constants.currentBot`.
9. When planning which factory calls (`Constants.createIntake(...)`, `Constants.createLimelight(...)`, etc.) succeed at runtime, verify which mechanisms the selected profile actually registers with `.addMechanism(...)`.

## Pathing conventions to follow

1. Put follower, drivetrain, and localizer configuration changes in `Constants.java` and `library/robot/...`, not in individual op modes.
2. For shared match-style autonomous code, keep geometry in an `IvyAuton_Paths` subclass and keep orchestration in `opModes/IvyAuton.java`.
3. Return a `PathChain` from autonomous builders, even when there is only one `Path`; `Test_IvyAuton.Paths#getPath1()` is the current example.
4. Set heading interpolation explicitly near the geometry.
5. In non-Ivy code, call `follower.update()` exactly once per loop after teleop or path-following commands are issued.

## `EquationCurve` / math-layer facts

- `EquationCurve` extends Pedro `CustomCurve`.
- Its constructor takes `startPose`, `endPose`, an `Equation`, and a `HeadingInterpolator`.
- It validates the endpoints with `MathFunctions.roughlyEquals(..., 1e-3)`.
- `t` is mapped linearly across the start and end `x` positions.
- Derivatives come from `equation.derivative()`.
- `Polynomial` is the only concrete `Equation` implementation currently present in TeamCode.
- `Polynomial.toString()` formats coefficients directly and is reasonable for telemetry/debug output.

## Current Ivy composition patterns in TeamCode

TeamCode currently composes Ivy behavior with:

- `Commands.instant(...)`
- `Commands.infinite(...)`
- `Commands.waitMs(...)`
- `Groups.sequential(...)`
- `Groups.loop(...)`
- `PedroCommands.follow(...)`
- `CommandConverter.toCommand(...)`

There is no current TeamCode example that implements `Command` directly, extends `CommandBuilder`, or constructs `new CommandBuilder()`. If you introduce those patterns, validate them against the `com.pedropathing:ivy:1.0.0` dependency instead of assuming a repo-local precedent exists.

## `java-autogen/` workflow

- `java-autogen/Ftc-PP-toJavaAuto-Ivy.html` accepts `.pp`, `.turt`, and `.json` inputs.
- `java-autogen/Ftc-PedroPathing-PathBuilder-toJavaAuto.html` is the browser-side builder/importer companion.
- The shared generator exposes a `useIvyCommandBase` setting and emits starter code that includes `reset()`, `schedule(...)`, and `Scheduler.execute()` scaffolding.
- Treat generated output as a scaffold: move it into the right TeamCode package, review headings/path names, and align it with the repo’s active `Constants.createFollower(...)` flow before relying on it.

## Repo-specific constraints

- Do not treat profile follower or localizer values as competition-tuned without verifying them on the configured robot.
- `LimelightPinpointLocalizer` is `@Deprecated` and is not wired into `RobotConstants.createFollower(...)`.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept test op mode that uses the standard follower plus raw Limelight color results.
- `tests/PreseasonWebcamAuto.java` gets its Limelight hardware name from `new Limelight.LIMELIGHT_PARAMS().HARDWAREMAP_NAME`.
- `learning/ExampleIvyWithSubsystems.java` is teaching code, not the profile-managed subsystem pattern: its `autoAlign()` step is commented out and its intake motor is resolved directly from `hardwareMap`.

## Rules

- Reuse the existing `Constants.createFollower(...)` → `RobotConstants.createFollower(...)` flow instead of constructing ad hoc followers elsewhere.
- Keep localization choices explicit: default code is Pinpoint-only, while Limelight fusion is opt-in and deprecated.
- Do not add duplicate `follower.update()` loops.
- Prefer the repo’s existing `Commands` / `Groups` / `PedroCommands` composition style in Ivy op modes.
- Keep non-drive subsystem behavior out of `RobotConstants`; use `RobotConstants` for profile data and mechanism registration.
- Reach for `EquationCurve` only when an equation-through-endpoints model is a better fit than Bezier geometry.
