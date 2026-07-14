---
name: library-utilities-and-shared-types
description: Use this skill when asked to extend lightweight shared types in `library/`, such as enums, timers, vectors, or small reusable helper classes.
---

# Library Utilities and Shared Types

Use this skill when changes belong in the lightweight `library/` package rather than in a mechanism or op mode.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/AllianceColor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
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
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
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
- `DriveVector.applyStickCurve(...)` mutates the instance and negates forward for stick-convention alignment; use `copy()` or `wStickCurve(...)` when a caller needs shaped input without mutating the original vector.
- `PIDFController<T>` and `DoublePIDFController` are generic control helpers used by proof-of-concept vision or tuning flows.
- `NoiseFilter` and `LowPassFilter` are reusable signal-conditioning helpers.
- `LinearInterp` and `LagrangeInterp` provide interpolation utilities that can be reused outside one op mode.
- `Equation` is the new shared math interface for reusable symbolic/value-returning curve expressions.
- `Polynomial` now lives under `library/math/`, implements `Equation`, and is still the only actually implemented reusable equation type in this package.
- `Direction` is the shared drive-direction enum used by `RobotConstants` and `Motor`, with helpers like `reverse()` and `toFtcSdk()`.
- `Motor` is still a low-level wrapper, but it now also implements `HardwareDevice` so it can integrate more naturally with FTC-style helper code.
- `OpModeHelper.kt` is a small op-mode utility for manual Lynx bulk caching plus loop `dt` timing; it enforces a simple state machine and promotes `STARTED -> LOOP` on the first `update()` call. Its unmodified Kotlin visibility is `public`, so it is available to other packages in the TeamCode module.
- `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator` are lower-level FTC hardware adapters that support direct `hardwareMap.get(...)` use cases.
- `Term` exists as early scaffolding for the broader `Equation` abstraction, but its symbolic operations still return placeholders and should not be treated as a finished utility yet.
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
- `toString()` now formats decimal coefficients safely with locale-stable fixed-point formatting instead of fragile substring truncation.
- Because it is in `library/math/`, keep it math-focused and free of FTC lifecycle assumptions.

## Equation-interface guidance

- Use `Equation` when code should accept a reusable math expression without caring whether the implementation is polynomial-backed.
- Right now, keep production geometry changes biased toward `Polynomial` inputs. `Polynomial` is the usable path for evaluation, derivatives, integrals, and polynomial-only arithmetic, but `getTerms()` and mixed-type `Equation` algebra still bottom out in unfinished `Term` scaffolding.
- If you extend the symbolic math layer, finish the real `Term` behavior instead of documenting or depending on placeholder `null` returns.

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
