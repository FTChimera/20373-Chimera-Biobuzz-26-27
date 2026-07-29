---
name: autonomous-path-patterns
description: Use this skill when asked to add or refactor autonomous geometry, `Path`/`PathChain` builders, or state-machine behavior in TeamCode autonomous op modes.
---

# Autonomous Path Patterns

Use this skill when the main task is autonomous geometry, path sequencing, or the control flow around path following.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/Blackboard.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Equation.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyWithSubsystems.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/kotlinTest/KtExampleIvyAuto.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Red_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Blue_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.html`
- `java-autogen/Ftc-PedroPathing-PathBuilder-toJavaAuto.html`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js`

## Autonomous styles that currently exist

1. Shared iterative Ivy sequencing: `opModes/IvyAuton.java` plus a variant-local `IvyAuton_Paths` implementation such as `opModes/variants/Test_IvyAuton.java`.
2. Standalone iterative Ivy teaching examples: `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt`.
3. Proof-of-concept sensor-following test code: `tests/PreseasonWebcamAuto.java`, which is a `LinearOpMode` test op mode that steers a follower from raw Limelight color results.

## Path-building conventions

- Use `PathChain` for the shared `IvyAuton_Paths` flow, even when there is only one `Path`.
- Use `BezierLine`, `BezierCurve`, or `EquationCurve` based on the geometry you actually need.
- Keep start pose and path geometry near each other.
- Set heading interpolation explicitly instead of relying on library defaults.
- If you use `EquationCurve`, ensure the chosen `Equation` passes through the start and end poses within the current `1e-3` rough-equality check and supply a `HeadingInterpolator`.

## Verified repo examples

- `Test_IvyAuton.Paths#getPath1()` returns `new PathChain(path1)` from a single `Path`.
- `tests/PedroPathingTeleOpTest.java` creates a path lazily with `follower.pathBuilder()` and a `BezierLine` that starts from `follower::getPose`.
- `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` each build two `PathChain`s and schedule them sequentially with `PedroCommands.follow(...)`.
- `tests/PreseasonWebcamAuto.java` uses the normal follower plus a generic `PIDFController<Pose>`; it does not use the shared `IvyAuton_Paths` base.
- `tests/PreseasonWebcamAuto.java` gets the Limelight hardware name from `new Limelight.LIMELIGHT_PARAMS().HARDWAREMAP_NAME` instead of inlining the string.
- `EquationCurve` is available for custom curve work, but no current TeamCode op mode or test instantiates it.
- `java-autogen/` can generate starter autonomous classes from `.pp`, `.turt`, or `.json` inputs; treat that output as scaffolding, not as a drop-in final routine.

## Mechanism creation rule for autonomous work

If autonomous work needs a new subsystem or vision wrapper, do not hide that construction inside path/state code.

Instead:

1. make the subsystem follow the repo’s `Mechanism` + `(HardwareMap, RobotConstants)` constructor pattern
2. register it in the selected robot profile with `.addMechanism(...)`
3. add or reuse a `Constants.create...(...)` helper when that improves readability
4. instantiate it during op mode initialization, not mid-path

The `OldDECODERobotTesting` and `FirstRobot` profiles exist in `RobotConfigurations`. Only mechanisms registered with `.addMechanism(...)` in the profile selected by `Constants.currentBot` are available through factory helpers; verify registration before relying on `Constants.create...()` calls in autonomous code.

## Rules

- Prefer path helpers or nested path classes over dumping raw coordinates into control-flow code.
- Keep transition conditions tied to follower state, command completion, or intentional timing points.
- If a path needs a specific heading profile, encode it in the builder next to the geometry.
- If the autonomous is variant-specific, keep geometry in the variant class and shared orchestration in the base.
- Keep reusable equation math in `library/math/...` and Pedro-specific curve wiring in `EquationCurve` or the path-builder layer.
- If a proof-of-concept test intentionally bypasses the normal wrapper, still source hardware names from `RobotConfigurations`, `Constants`, or the owning `*_PARAMS` class instead of duplicating literals.

## Things to watch

- `tests/PreseasonWebcamAuto.java` is test code, not the shared match-autonomous architecture.
- `opModes/IvyAuton.java` stores `follower.getPose()` into `Blackboard.Caption.POSE` in `stop()`, and `variants/Red_TeleOp.java` / `Blue_TeleOp.java` read that pose back if it is not `(0,0,0)`.
- `learning/ExampleIvyWithSubsystems.java` is a teaching scaffold with its `autoAlign()` step commented out; do not copy its direct intake `DcMotor` construction as the repo-standard subsystem pattern.
