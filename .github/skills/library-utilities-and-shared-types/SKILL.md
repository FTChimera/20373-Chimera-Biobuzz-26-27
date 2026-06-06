---
name: library-utilities-and-shared-types
description: Use this skill when asked to extend lightweight shared types in `library/`, such as enums, timers, vectors, or small reusable helper classes.
---

# Library Utilities and Shared Types

Use this skill when changes belong in the lightweight `library/` package rather than in a mechanism or op mode.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/AllianceColor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/Timer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
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
- `Polynomial` is a lightweight reusable math helper with evaluation, scaling, multiplication, addition, and derivative support.
- `Direction` is the shared drive-direction enum used by `RobotConstants` and `Motor`, with helpers like `reverse()` and `toFtcSdk()`.
- `Timer` and hardware wrappers support op modes and mechanisms without owning robot behavior.
- `library/robot/RobotConstants` is a configuration object rather than an active subsystem.

## Boundary with the new mechanism pattern

- If a change adds robot behavior, it probably belongs in `mechanisms/`, not in `library/`.
- If a wrapper in `library/hardware/...` participates in the robot-profile creation flow, it should usually extend `Mechanism` and accept `(HardwareMap, RobotConstants)`.
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
- Add math helpers such as polynomial evaluation or transformation logic that can be reused outside one op mode.
- Add a wrapper or adapter that makes raw FTC devices easier to use from mechanisms.
- Add shared conversion helpers like the `Direction` bridge between repo-level configuration and FTC SDK device APIs.

## Polynomial-specific guidance

- `Polynomial` stores coefficients highest-degree first.
- `getY(x)` evaluates the polynomial at `x`.
- `derivative()` returns a new `Polynomial`, which is what `EquationCurve` uses for derivative and curvature calculations.
- Because it is in `library/`, keep it math-focused and free of FTC lifecycle assumptions.

## Relationship to `EquationCurve`

- `EquationCurve` lives in `pedroPathing/custom/`, not `library/`, because it adapts repo math helpers into Pedro’s `CustomCurve` API.
- Use `Polynomial` here when the request is about the reusable math primitive.
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

