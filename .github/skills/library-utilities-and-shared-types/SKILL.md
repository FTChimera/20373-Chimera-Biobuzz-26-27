---
name: library-utilities-and-shared-types
description: Use this skill when asked to extend lightweight shared types in `library/`, such as enums, timers, vectors, or small reusable helper classes.
---

# Library Utilities and Shared Types

Use this skill when changes belong in the lightweight `library/` package rather than in a mechanism or op mode.

## Upstream `~/.gradle` lookup

If a helper in `library/` exists mainly to adapt an external dependency, inspect
the downloaded module under `~/.gradle/caches/modules-2/files-2.1/` with the
version pinned in `build.dependencies.gradle`.

- `com.pedropathing/core` and `com.pedropathing/ftc` for Pedro math, pose, and
  pathing behavior
- `com.pedropathing/ivy` for command-adapter helpers
- `org.firstinspires.ftc/RobotCore`, `org.firstinspires.ftc/Hardware`, and
  sometimes `org.firstinspires.ftc/Vision` for SDK-backed utility behavior

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/AllianceColor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/game/GameElement.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/game/Pollen.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/Timer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/control/PIDFController.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/control/DoublePIDFController.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/control/DoublePIDFCoeffs.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/filters/NoiseFilter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/filters/LowPassFilter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Motor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/RevPotentiometer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/SlidesMotor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/RGBIndicator.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/interpolation/LinearInterp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/interpolation/LagrangeInterp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Equation.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/MathUtils.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/SumEquation.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Term.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/...`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`

## Design intent

- Keep the library layer small and reusable.
- Put value objects, small helpers, enums, and hardware adapters here.
- Avoid turning `library/` into a second subsystem layer.

## Current examples

- `AllianceColor` carries alliance-level intent for shared op mode bases.
- `DriveVector` is a lightweight drive math object used by the drivetrain stack.
- `DriveVector.applyStickCurve(...)` mutates the instance without changing axis signs; the drive stack's forward inversion happens later in `MecanumDrive`.
- `GameElement` and `Pollen` are lightweight preseason game-piece marker types under `library/game/`.
- `PIDFController<T>` and `DoublePIDFController` are generic control helpers used by proof-of-concept vision or tuning flows.
- `NoiseFilter` and `LowPassFilter` are reusable signal-conditioning helpers.
- `LinearInterp` and `LagrangeInterp` provide interpolation utilities that can be reused outside one op mode.
- `MathUtils.kt` is the shared home for unit conversion, heading normalization, FTC/Pedro pose-conversion helpers, and planner geometry helpers such as `adaptiveHandleLengths(...)`, `maxTurnAngle(...)`, and `maxSpacingRatio(...)`.
- `Equation` is the shared math interface for reusable symbolic/value-returning curve expressions.
- `Polynomial` now lives under `library/math/`, implements `Equation`, and still stores coefficients highest-degree first.
- `SumEquation` is the internal normalized representation returned by `Equation.of(...)`; it combines like terms and drops zero terms instead of leaving callers with ad hoc lists.
- `Direction` is the shared drive-direction enum used by `RobotConstants` and `Motor`, with helpers like `reverse()` and `toFtcSdk()`.
- `Motor` is still a low-level wrapper, but it now also implements `HardwareDevice` so it can integrate more naturally with FTC-style helper code.
- `OpModeHelper.kt` is a small op-mode utility for manual Lynx bulk caching plus loop `dt` timing; it enforces a simple state machine, keeps `dt` as a `Double` in milliseconds, and promotes `STARTED -> LOOP` on the first `update()` call. Its unmodified Kotlin visibility is `public`, so it is available to other packages in the TeamCode module.
- `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator` are lower-level FTC hardware adapters that support direct `hardwareMap.get(...)` use cases.
- `Term` is now a real immutable `Equation` implementation for constants, powers, and several elementary functions (`sin`, `cos`, `tan`, `exp`, and related integral-only helper forms), with derivative/product-rule behavior and explicit `UnsupportedOperationException` for unsupported antiderivatives.
- `Timer` is a unit-aware helper whose `get(...)`, `dt(...)`, and `past(...)` APIs now return or accept `double` values in the requested `TimeUnit`.
- `Timer` and hardware wrappers support op modes and mechanisms without owning robot behavior.
- `library/robot/RobotConstants` is a configuration object rather than an active subsystem.

## Boundary with the new mechanism pattern

- If a change adds robot behavior, it probably belongs in `mechanisms/`, not in `library/`.
- If a wrapper in `library/hardware/...` participates in the robot-profile creation flow, it should usually extend `Mechanism` and accept `(HardwareMap, RobotConstants)`.
- If a wrapper is a lower-level adapter like `Motor`, it can stay outside the `Mechanism` hierarchy and still be reused by mechanisms.
- If a class is a custom FTC `HardwareDevice` under `library/hardware/hardwareDevice/`, keep it focused on SDK device behavior rather than profile registration or mechanism orchestration.
- Keep robot-profile state in `RobotConstants` / `RobotConfigurations`, not in generic utility classes.
- Do not move mechanism behavior into `library/` just because it needs a helper type.

In other words: keep `library/` focused on small reusable types. Use `Mechanism` subclasses and `RobotConstants.createMechanism(...)` when the class starts behaving like a robot subsystem.

## Rules

- Prefer small, focused types.
- If a class starts owning physical behavior, move that behavior into `mechanisms/`.
- If a helper depends heavily on FTC lifecycle or telemetry, question whether it belongs in an op mode instead.
- Keep shared types dependency-light when practical so they remain broadly reusable across TeamCode.
- Keep drivetrain/follower profile data in `library/robot/RobotConstants`; keep Pedro-specific curve adapters outside the generic library layer.

## Good use cases

- Add a small enum, value object, or utility method.
- Add helper behavior to `DriveVector` or another reusable math/configuration class.
- Add math helpers such as interpolation, polynomial evaluation, or transformation logic that can be reused outside one op mode.
- Add reusable control helpers that are still generic and not tied to one specific subsystem implementation.
- Add reusable filtering helpers for noisy sensor or signal data.
- Add a wrapper or adapter that makes raw FTC devices easier to use from mechanisms.
- Add a reusable op-mode helper like `OpModeHelper` when the behavior is generic lifecycle plumbing rather than robot-specific control logic.
- Add shared conversion helpers like the `Direction` bridge between repo-level configuration and FTC SDK device APIs.

## Polynomial-specific guidance

- `Polynomial` stores coefficients highest-degree first.
- `getY(x)` evaluates the polynomial at `x`.
- `derivative()` returns a new `Polynomial`, which still satisfies the current `EquationCurve` derivative/curvature contract.
- `add(Polynomial)` and `multiply(Polynomial)` preserve polynomial results, while mixed `Equation` arithmetic widens through `Equation.of(getTerms())`.
- Because it is in `library/math/`, keep it math-focused and free of FTC lifecycle assumptions.

## Equation-interface guidance

- Use `Equation` when code should accept a reusable math expression without caring whether the implementation is polynomial-backed.
- `Equation.of(...)` is the repo’s normalization entry point when you need a sum of symbolic terms instead of a polynomial-only type.
- Mixed polynomial/trigonometric algebra is now supported through `Equation`, `Term`, and `SumEquation`, but unsupported antiderivatives should still fail explicitly with `UnsupportedOperationException` rather than being papered over.
- If you extend the symbolic math layer, keep `getTerms()`, derivative behavior, and integral failures internally consistent with the existing `EquationMathTest` coverage.

## Interpolation-specific guidance

- `LinearInterp` is the lightweight piecewise-linear helper for table-based lookup.
- `LagrangeInterp` builds a reusable `Polynomial` from known `(x, y)` samples.
- If an interpolation task is still fundamentally robot-agnostic math, keep it here instead of moving it into an op mode or mechanism.

## Control/filter guidance

- `PIDFController<T>` should stay generic; if a change starts depending on one specific mechanism’s lifecycle or telemetry, that logic likely belongs higher up.
- `PIDFController<T>` now has both `atTarget()` and `atTarget(double[] error)` so callers and internal updates can evaluate the current error vector against configured tolerances before zeroing output.
- `DoublePIDFCoeffs` is the lightweight bridge for one-dimensional PIDF use cases.
- `PIDFCoefficients<T>` now supports scalar broadcasting for multi-dimensional state types more safely: scalar `setPIDF(...)`, `setTolerance(...)`, and `setIntegralLimit(...)` resolve the configured state dimension or infer it from the suppliers instead of trying to build a state from a zero-length array.
- `NoiseFilter` and `LowPassFilter` should stay algorithm-focused; avoid hiding robot-specific state machines inside them.

## Relationship to `EquationCurve`

- `EquationCurve` lives in `pedroPathing/custom/`, not `library/`, because it adapts repo math helpers into Pedro’s `CustomCurve` API.
- Use `Polynomial` here when the request is about the reusable math primitive.
- Use `Equation` here when the request is about the shared expression interface or widening a helper to work with more than just polynomials.
- Use `MathUtils` here when the change is fundamentally about generic conversion or heading math rather than path construction.
- Use the Pedro pathing skills when the request is about turning that math into followable path geometry.

## Direction-specific guidance

- For drive or wrapper configuration that should stay FTC-SDK-agnostic, prefer `library.hardware.actuators.Direction` over threading `DcMotorSimple.Direction` through higher-level config objects.
- Keep conversion to FTC SDK enums at the boundary, such as `Direction.toFtcSdk()` inside wrappers or drivetrain-profile builders.
- If a change only needs drive-direction intent, extend `Direction` or reuse it rather than creating a second repo-local forward/reverse enum.

## Avoid

- Putting match-specific routines in `library/`.
- Hiding robot state machines in utility classes.
- Adding large framework abstractions when a small helper class would do.
- Using `library/` as a dumping ground for subsystem logic that should instead live behind `Mechanism` and `RobotConstants.createMechanism(...)`.
