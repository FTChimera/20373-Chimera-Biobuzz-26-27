---
name: tests-and-tuning-opmodes
description: Use this skill when asked to add or modify test op modes, Pedro tuning flows, or validation helpers under `tests/` and `pedroPathing/Tuning.java`.
---

# Tests and Tuning OpModes

Use this skill when work is focused on validation op modes, tuners, or measurement workflows instead of match-ready robot behavior.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MotorDirectionTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`

## Current repo patterns

- Test op modes live under `tests/` and are annotated for the driver station.
- Pedro tuning is centralized in `pedroPathing/Tuning.java`, which exposes many selectable tuner sub-modes.
- Tuning flows rely on `Constants.createFollower(hardwareMap)` so they inherit the current robot profile and follower config.
- Other subsystems used in tests should also come from `Constants.create...(...)` so tests exercise the same construction paths as match code.

## Existing examples

- `MecanumTeleOp` is a raw FTC iterative test op mode with robot-centric mode, field-centric mode, IMU yaw reset, and a built-in motor-direction test mode.
- `MotorDirectionTest` is a simple low-speed sanity check for the constants-driven drivetrain profile and intake direction.
- `PedroPathingTeleOpTest` mixes manual drive with on-demand path following for follower behavior checks.
- `Tuning.java` contains localization, velocity, braking, PIDF, centripetal, and path-shape tuners.
- `EquationCurve` plus `Polynomial` are now part of the custom path-shape toolbox if a test or tuner needs non-Bezier geometry.

## Rules

- Keep validation op modes safe by using low power or explicit warnings when motors may move unexpectedly.
- Reuse the repo’s existing follower creation path so tuning matches real robot configuration.
- Reuse centralized subsystem factories from `Constants` so tests do not drift from production wiring.
- Put one clear purpose per test op mode.
- When adding a tuner, explain what the driver should physically do and what value they should record.
- If testing custom path geometry, make it explicit whether the path uses Bezier primitives or `EquationCurve` / `Polynomial` math.

## Important repo notes

- Many tuning values in the robot profile are still placeholders.
- Real robot measurements matter more than code-level plausibility here.
- Some tuner code comes from Pedro tooling and is intentionally verbose; keep repo-specific adjustments minimal and clear.
- If a new mechanism is being validated, make sure it follows the same `Mechanism` + `RobotConstants.createMechanism(...)` pattern before adding the test.

## Validation expectations

- Build after changing tuning or test classes.
- Prefer a robot-side sanity check before claiming tuning-related behavior is correct.
- If a test touches drive direction, check it with the robot safely off the ground first when practical.
- Use `MecanumTeleOp` when you need direct SDK-level drive and IMU behavior checks; use `MotorDirectionTest` when you want to validate the current `Constants` / `RobotConfigurations` drivetrain wiring at low speed.

