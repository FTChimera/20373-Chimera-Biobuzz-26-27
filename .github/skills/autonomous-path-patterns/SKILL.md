---
name: autonomous-path-patterns
description: Use this skill when asked to add or refactor autonomous geometry, `Path`/`PathChain` builders, or state-machine behavior in TeamCode autonomous op modes.
---

# Autonomous Path Patterns

Use this skill when autonomous work is mainly about path definitions and sequencing.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Equation.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/kotlinTest/KtExampleIvyAuto.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/learning/ExampleIvyWithSubsystems.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.html`
- `java-autogen/Ftc-PedroPathing-PathBuilder-toJavaAuto.html`
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.js`

## Two autonomous styles in this repo

1. Ivy-driven sequencing, as in `IvyAuton`, where a variant supplies `IvyAuton_Paths` and the shared base schedules Pedro commands.
2. Standalone proof-of-concept loops, as in `tests/PreseasonWebcamAuto`, where a `LinearOpMode` drives a follower from sensor feedback instead of using an `IvyAuton_Paths` variant.

## Path-building conventions

- Use `PathChain` for Ivy autonomous builders, even when there is only one `Path`.
- Use `BezierLine`, `BezierCurve`, or `EquationCurve` depending on whether the motion is straight, control-point-based, or easiest to express as `y = f(x)`.
- Keep start pose and path geometry near each other.
- Set heading interpolation explicitly rather than relying on implicit defaults.
- If using `EquationCurve`, make sure the chosen `Equation` passes through the start and end poses within the current rough-equality tolerance (`1e-3`) and provide an explicit `HeadingInterpolator`.

## Repo examples

- `Test_IvyAuton.Paths` returns a `PathChain` built from one `Path`.
- `tests/PreseasonWebcamAuto` is a color-tracking proof of concept that uses a follower plus a generic `PIDFController<Pose>` rather than a reusable autonomous base class.
- Even though `PreseasonWebcamAuto` still uses raw `Limelight3A` access, it now pulls the camera hardware name from `Limelight.LIMELIGHT_PARAMS` instead of a stale hardcoded test-only string.
- `PedroPathingTeleOpTest` uses `follower.pathBuilder()` lazily to create a path from the current pose.
- `learning/ExampleIvyAuto` is a compact iterative Ivy autonomous example that builds two `PathChain`s, then schedules them sequentially with `PedroCommands.follow(...)`; `learning/kotlinTest/KtExampleIvyAuto.kt` is its Kotlin companion.
- `java-autogen/` can generate starter autonomous classes from Pedro/Turtle Tracer exports or builder-authored drafts; use that output as a scaffold, not as a substitute for reviewing repo conventions.
- `EquationCurve` plus the current `Equation`/`Polynomial` math layer are available for custom curve work even if no current autonomous routine uses them heavily yet.

## Mechanism creation rule for autonomous work

If autonomous needs a new subsystem or vision wrapper, do not construct it ad hoc inside the path/state code.

Instead:

1. make the subsystem follow the repo’s `Mechanism` + `(HardwareMap, RobotConstants)` constructor pattern
2. register it in the active robot profile with `.addMechanism(...)`
3. add or reuse a `Constants.create...(...)` helper when that improves readability
4. instantiate through that normal creation path during op mode initialization

That keeps autonomous and teleop using the same subsystem construction path.

## Rules

- Prefer path helpers or nested path classes over dumping raw coordinates directly into op mode control flow.
- Keep transition conditions tied to follower state like `isBusy()` or timing, not arbitrary delays alone.
- If a path needs a specific heading profile, encode it in the path builder next to the geometry.
- If the autonomous is variant-specific, put geometry in the variant class and keep shared orchestration in the base.
- Create mechanisms through `Constants`, not direct constructors sprinkled through state-machine branches.
- Keep reusable equation math helpers in `library/` and Pedro-specific curve wiring in `EquationCurve` or the path builder layer.
- If a proof-of-concept test intentionally bypasses the normal wrapper, still source hardware names from `RobotConfigurations` or the owning `*_PARAMS` class instead of introducing duplicate literals.

## Things to watch

- `tests/PreseasonWebcamAuto` is a test/proof-of-concept op mode, not a shared match autonomous architecture example.
- Follower-localization choice matters if the camera pipeline is needed for vision tasks.
- Preserve pose handoff if the autonomous is expected to feed teleop state.
- `learning/ExampleIvyWithSubsystems.java` is a teaching scaffold with its unfinished `autoAlign()` step commented out; do not use its direct intake `DcMotor` construction as an autonomous architecture template.
