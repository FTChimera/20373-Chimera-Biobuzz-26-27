---
name: library-utilities-and-shared-types
description: Use this skill for small reusable types and utilities under library.
---

# Library Utilities and Shared Types

Keep shared utilities small, explicit, and independent of FTC entry-point lifecycle.

## Java math library

- The math utility source is `library/math/MathUtils.java`.
- It is Java, not Kotlin.
- It provides shared unit conversion, heading normalization, distances, and pose helpers.
- It also provides planner geometry helpers used by intake path generation.
- Reuse it instead of duplicating conversion or angle-wrap logic.
- Preserve the coordinate-system argument where a conversion depends on conventions.

## Math changes

- Keep pure math deterministic and free of `HardwareMap` access.
- Document units for input and output values.
- Define behavior at zero, wrap boundaries, and invalid numeric inputs.
- Use existing MathUtils tests when changing established operations.
- Add a focused existing test pattern for newly public deterministic helpers.
- Do not encode robot-profile calibration in generic math.

## State and mode

- `library/game/State` carries runtime context for mechanisms.
- It includes the current robot mode, robot, helper, alliance/mode tuple, and bindings.
- `Robot.periodic(State)` populates aggregate-owned context before mechanism dispatch.
- The state mode must agree with the aggregate robot mode.
- Keep mode-specific behavior in mechanisms or lifecycle code, not in generic utilities.

## Shared planner data

- `library/planners/Common` groups shared timings, tuning values, and alliance poses.
- Its nested pose groups use `AlliancePoses`.
- Call `Common.init(alliance)` to select the active alliance side.
- Default zero poses are scaffolding until calibrated.
- Do not convert a shared placeholder into a hidden hard-coded route constant.

## Vision path utility

- `IntakeBallPath` is a reusable vision-driven intake-path generator.
- The full overload validates detections and returns `null` when no safe route exists.
- It filters detector confidence, area, range, and lateral offset.
- It uses `MathUtils` for distance, headings, and robot-to-field offsets.
- The no-argument generator returns an empty `PathChain` scaffold.
- Keep detector and FTC SDK types at the public boundary rather than leaking them through generic math.

## Simple reusable types

- Prefer dedicated enums and compact data types for stable domain concepts.
- Keep mutable global state out of utility classes.
- Use `Timer` for time measurements where existing code uses its units-aware API.
- Preserve nullability contracts and fail fast on invalid mandatory inputs.
- Avoid adding a utility solely to hide a one-line local expression.

## Hardware boundary

- Device wrappers and mechanisms belong outside general utilities.
- A shared utility may depend on geometry or basic Java types.
- It should not create motors, cameras, or FTC op mode instances.
- Place profile-specific constants in robot configuration classes.
- Place lifecycle behavior in `Robot` or a mechanism.

## Validation

- Compile Java library changes with `:TeamCode:assembleDebug`.
- Run `:TeamCode:test` when available for utility behavior.
- Test angle wrapping and coordinate conversions near boundaries.
- Test null and empty input behavior for path utilities.
- Check callers before changing a public utility signature.

## API design

- Name methods for their unit, frame, or side effect when ambiguity is possible.
- Return a documented sentinel only when callers can safely distinguish it.
- Prefer immutable return data for shared coordinate and configuration values.
- Avoid convenience overloads that silently choose a coordinate system.
- Keep public constants final when they represent fixed behavior.
- Make temporary scaffolds obvious in names and documentation.
- Preserve binary and source compatibility when a broad utility has many callers.
- Search all production and test callers before removing a public helper.

## Documentation review

- Cite `MathUtils.java` as the current math source.
- Cite the full intake-path overload for detector-driven behavior.
- Label empty-path scaffolds and default poses as unfinished where applicable.
- Do not claim a utility owns FTC lifecycle or hardware construction.
- Keep shared-type guides aligned with the aggregate robot lifecycle.

## Change boundaries

- Keep Java math independent of robot profile selection.
- Keep route-specific values with planners rather than general utilities.
- Keep SDK and hardware lifecycle behavior in their dedicated layers.
- Recheck imports and callers when moving a reusable type between packages.
