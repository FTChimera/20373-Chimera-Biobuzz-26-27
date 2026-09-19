---
name: constants-not-configured
description: Use this skill for provisional robot, field, and game values.
---

# Constants Not Configured

Use this guide when source has an intentional placeholder rather than a tuned value.

## Profile selection

- `Constants.currentBot` is a compile-time field assignment.
- It currently assigns `RobotConfigurations.FirstRobot()`.
- Selecting another profile requires editing `Constants.java` and rebuilding.
- Do not describe the profile as freely or permanently selectable at runtime.
- A helper on `Constants` does not prove the active profile has that mechanism.
- Check profile registration before using a mechanism factory or `Robot.require(...)`.

## Profile-managed mechanisms

- `RobotConstants` owns hardware names, drivetrain settings, and mechanism registration.
- `Constants.createRobot(hardwareMap, mode)` creates the aggregate robot.
- `Robot` constructs only the mechanism classes registered by the active profile.
- `Robot.get(...)` can return `null`.
- `Robot.require(...)` fails when the mechanism is absent.
- New code must choose between optional behavior and an explicit requirement.
- Use `Mechanism.Params` and `resolveParams(...)` for nested profile parameters.

## Current placeholders

- `FirstRobot()` retains zero values for velocity-related tuning.
- Its heading PIDF values remain provisional.
- Its centripetal-scaling fields remain provisional.
- A zero does not establish a valid competition tuning.
- Document the expected physical measurement and calibration procedure beside a new placeholder.
- Do not silently substitute guessed measurements for declared placeholders.

## Field and game data

- Verify field dimensions, alliance coordinates, and scoring geometry from current source.
- `FieldUpdates.getGoalPose()` currently returns `null`.
- Treat that return as unfinished behavior, not a valid target location.
- `Common` contains shared planner timing and pose placeholders.
- Its pose groups start with default `Pose` instances until calibrated.
- `Common.init(alliance)` selects active alliance values but does not calibrate them.
- Keep provisional geometry isolated from mechanism control logic.

## Autonomous values

- The current concrete autonomous is `autos/Garden8`.
- It composes routines using `PlanBuilder`.
- Garden-side planner transitions are implemented only where their source module exists.
- Loading-side transitions marked incomplete must remain documented as incomplete.
- `IntakeBallPath` consumes camera detections only through its full-argument overload.
- Its no-argument overload returns an empty `PathChain` scaffold.
- Vision thresholds and distance scale in that class are tunables, not field guarantees.

## Entry-point defaults

- `tele/TeleOp` gets alliance and pose from `Blackboard` when available.
- It otherwise uses the red default start pose after defaulting alliance to red.
- Nested `Red_Tele` and `Blue_Tele` entries set the selected alliance.
- `Garden8` has nested red and blue annotated autonomous entries.
- Keep start-pose defaults consistent with Blackboard handoff behavior.

## Safe editing process

1. Locate every consumer before changing a value.
2. Determine whether the value is a unit conversion, hardware calibration, or game geometry.
3. Preserve units in the name or accompanying documentation.
4. Confirm coordinate-system conventions before changing a pose.
5. Update profile parameters rather than hard-coding a competing value in a mechanism.
6. Add or update an existing test when the value participates in deterministic math.
7. Build the TeamCode module after Java source changes.

## Review checklist

- Is the new value measured, specified, or intentionally temporary?
- Does the active profile actually consume it?
- Are red and blue values both valid?
- Does the change alter a path endpoint or heading convention?
- Does optional hardware remain optional?
- Does the documentation accurately label unfinished behavior?
- Has the configured profile been rebuilt after changing `Constants`?

## Handoff and configuration boundaries

- Blackboard values are runtime handoff state, not calibration constants.
- Robot profile fields describe the selected hardware and control configuration.
- Planner values describe a route and should not silently override a profile.
- Keep camera mounting data with the vision configuration that owns it.
- Keep mechanism power and controller values with their mechanism parameters.
- Check source consumers before moving a value between these locations.

## Evidence for a final value

- Prefer measured values with a recorded unit and test context.
- Use field documentation for game geometry rather than visual estimates.
- Verify motor directions and encoder signs on hardware.
- Verify a new heading value against the active coordinate convention.
- Confirm an alliance-specific value has a deliberate counterpart.
- Retain a clearly named provisional value when evidence is unavailable.
