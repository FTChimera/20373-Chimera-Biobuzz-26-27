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
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/...`

## Design intent

- Keep the library layer small and reusable.
- Put value objects, small helpers, enums, and hardware adapters here.
- Avoid turning `library/` into a second subsystem layer.

## Current examples

- `AllianceColor` carries alliance-level intent for shared op mode bases.
- `DriveVector` is a lightweight drive math object used by the drivetrain stack.
- `Timer` and hardware wrappers support op modes and mechanisms without owning robot behavior.
- `library/robot/RobotConstants` is a configuration object rather than an active subsystem.

## Rules

- Prefer small, focused types.
- If a class starts owning physical behavior, move that behavior into `mechanisms/`.
- If a helper depends heavily on FTC lifecycle or telemetry, question whether it belongs in an op mode instead.
- Keep shared types dependency-light when practical so they remain broadly reusable across TeamCode.

## Good use cases

- Add a small enum, value object, or utility method.
- Add helper behavior to `DriveVector` or another reusable math/configuration class.
- Add a wrapper or adapter that makes raw FTC devices easier to use from mechanisms.

## Avoid

- Putting match-specific routines in `library/`.
- Hiding robot state machines in utility classes.
- Adding large framework abstractions when a small helper class would do.

