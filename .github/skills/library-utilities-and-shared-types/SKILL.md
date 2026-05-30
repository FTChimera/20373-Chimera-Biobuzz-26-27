---
name: library-utilities-and-shared-types
description: Use this skill when asked to extend lightweight shared types in `library/`, such as enums, timers, vectors, or small reusable helper classes.
---

# Library Utilities and Shared Types

Use this skill when changes belong in the lightweight `library/` package rather than in a mechanism or op mode.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/AllianceColor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/Timer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/...`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`

## Design intent

- Keep the library layer small and reusable.
- Put value objects, small helpers, enums, and hardware adapters here.
- Avoid turning `library/` into a second subsystem layer.

## Current examples

- `AllianceColor` carries alliance-level intent for shared op mode bases.
- `DriveVector` is a lightweight drive math object used by the drivetrain stack.
- `Direction` is the shared drive-direction enum used by `RobotConstants` and `Motor`, with helpers like `reverse()` and `toFtcSdk()`.
- `Timer` and hardware wrappers support op modes and mechanisms without owning robot behavior.
- `library/robot/RobotConstants` is a configuration object rather than an active subsystem.

## Boundary with the new mechanism pattern

- If a change adds robot behavior, it probably belongs in `mechanisms/`, not in `library/`.
- If a wrapper in `library/hardware/...` needs configurable fields, give the wrapper its own params type.
- Keep the shared editable params instance for that wrapper in `Constants`.
- Do not move mechanism params into generic utility classes just because they are shared.

In other words: the type definition can live with the wrapper, but the live shared config instance should usually be centralized in `Constants`.

## Rules

- Prefer small, focused types.
- If a class starts owning physical behavior, move that behavior into `mechanisms/`.
- If a helper depends heavily on FTC lifecycle or telemetry, question whether it belongs in an op mode instead.
- Keep shared types dependency-light when practical so they remain broadly reusable across TeamCode.
- Keep drivetrain/follower profile data in `library/robot/RobotConstants`; keep non-drive subsystem params surfaced centrally through `Constants`.

## Good use cases

- Add a small enum, value object, or utility method.
- Add helper behavior to `DriveVector` or another reusable math/configuration class.
- Add a wrapper or adapter that makes raw FTC devices easier to use from mechanisms.
- Add shared conversion helpers like the `Direction` bridge between repo-level configuration and FTC SDK device APIs.

## Direction-specific guidance

- For drive or wrapper configuration that should stay FTC-SDK-agnostic, prefer `library.hardware.actuators.Direction` over threading `DcMotorSimple.Direction` through higher-level config objects.
- Keep conversion to FTC SDK enums at the boundary, such as `Direction.toFtcSdk()` inside wrappers or drivetrain-profile builders.
- If a change only needs drive-direction intent, extend `Direction` or reuse it rather than creating a second repo-local forward/reverse enum.

## Avoid

- Putting match-specific routines in `library/`.
- Hiding robot state machines in utility classes.
- Adding large framework abstractions when a small helper class would do.
- Using `library/` as a dumping ground for subsystem params instances that should instead be exposed through `Constants`.

