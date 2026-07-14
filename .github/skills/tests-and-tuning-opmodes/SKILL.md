---
name: tests-and-tuning-opmodes
description: Use this skill when asked to add or modify test op modes, Pedro tuning flows, FTC AutoTune op modes, or validation helpers under `tests/`, `pid/autoTune/`, and `pedroPathing/Tuning.java`.
---

# Tests and Tuning OpModes

Use this skill when work is focused on validation op modes, tuners, or measurement workflows instead of match-ready robot behavior.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MotorDirectionTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/androidTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/MotorTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/RGBIndicatorTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/SlidesMotorTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Slides1.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/TuningConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/PositionPIDTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/VelocityPIDFTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/DualVelocityPIDFTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `build.dependencies.gradle`

## Current repo patterns

- Test op modes live under `tests/` and are annotated for the driver station.
- `tests/ftcSdk/androidTest.java` is a shared `LinearOpMode` base for simple SDK-level validation flows that run one scripted check, sleep briefly, and then request stop.
- `tests/ftcSdk/` now also includes custom-hardware checks for the device-level `RGBIndicator` and `SlidesMotor` classes under `library/hardware/hardwareDevice/`.
- `pid/autoTune/` now contains FTC AutoTune relay-feedback tuners built around repo-local `pid.autoTune.master.PIDMaster` / `DualMotorPIDMaster` FTC adapters, the external `pidautotuner-core` algorithms, and a repo-local `@Configurable` `TuningConstants` class.
- Those FTC AutoTune op modes are driver-station-visible `@TeleOp` entries in group `FtcAutoTune`, and they are calibration tools rather than match-ready robot behavior.
- Pedro tuning is centralized in `pedroPathing/Tuning.java`, which exposes many selectable tuner sub-modes.
- Tuning flows rely on `Constants.createFollower(hardwareMap)` so they inherit the current robot profile and follower config.
- Other subsystems used in tests should also come from `Constants.create...(...)` so tests exercise the same construction paths as match code.

## Existing examples

- `MecanumTeleOp` is a raw FTC iterative test op mode with robot-centric mode, field-centric mode, IMU yaw reset, and a built-in motor-direction test mode.
- `MotorDirectionTest` is a looping low-speed sanity check for the constants-driven drivetrain profile and intake direction, and it now updates telemetry live while you flip directions.
- `PedroPathingTeleOpTest` mixes manual drive with on-demand path following for follower behavior checks.
- `PreseasonWebcamAuto` is a proof-of-concept color-tracking `LinearOpMode` that uses a follower plus raw Limelight results.
- `MotorTest` is a small SDK-level wrapper check built on the custom `androidTest` base.
- `RGBIndicatorTest` validates both the device-level `library.hardware.hardwareDevice.RGBIndicator` and the profile-style `library.hardware.leds.RGBIndicator` wrapper.
- `SlidesMotorTest` is a direct SDK-level validation flow for the custom `SlidesMotor` hardware device, and its hardware name now comes from `Slides1.SLIDES1_PARAMS`.
- `PositionPIDTunerOpMode`, `VelocityPIDFTunerOpMode`, and `DualVelocityPIDFTunerOpMode` are thin wrappers around the FTC AutoTune library that run relay-feedback calibration plus optional `gamepad1.a` live tests.
- `TuningConstants` is the shared place for FTC AutoTune motor names, direction booleans, target RPM / ticks, relay amplitude, and feedforward sweep settings.
- `Tuning.java` contains localization, velocity, braking, PIDF, centripetal, and path-shape tuners.
- `EquationCurve` plus `Polynomial` are now part of the custom path-shape toolbox if a test or tuner needs non-Bezier geometry.
- `PreseasonWebcamAuto` also exercises the generic `PIDFCoefficients<Pose>` scalar-broadcast path through `setTolerance(0.5)` and `setIntegralLimit(1.0)`.

## Rules

- Keep validation op modes safe by using low power or explicit warnings when motors may move unexpectedly.
- Reuse the repo’s existing follower creation path so tuning matches real robot configuration.
- Reuse centralized subsystem factories from `Constants` so tests do not drift from production wiring.
- Treat `pid/autoTune/` as an intentional exception to the normal subsystem-factory rule: those op modes pass raw motor names and directions through `TuningConstants` because the repo-local master adapters own the FTC relay-test integration.
- If a test intentionally bypasses the normal factory path, still source hardware names from `RobotConfigurations` or the owning nested `*_PARAMS` class instead of introducing standalone literals.
- Put one clear purpose per test op mode.
- When adding a tuner, explain what the driver should physically do and what value they should record.
- For FTC AutoTune op modes, document the two-phase flow (automatic relay test, optional live test) and call out any hard-stop or flywheel spin-up risks before the driver presses Start.
- If testing custom path geometry, make it explicit whether the path uses Bezier primitives or `EquationCurve` / `Polynomial` math.
- If a test intentionally bypasses the normal wrapper flow, document that it is a proof-of-concept or SDK-level validation mode.

## Important repo notes

- Many tuning values in the robot profile are still placeholders.
- Real robot measurements matter more than code-level plausibility here.
- `build.dependencies.gradle` now resolves JitPack plus `com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.3.7`; the removed FTC artifact is replaced by repo-local master adapters, so tuning/build work should keep both layers aligned.
- Some tuner code comes from Pedro tooling and is intentionally verbose; keep repo-specific adjustments minimal and clear.
- If a new mechanism is being validated, make sure it follows the same `Mechanism` + `RobotConstants.createMechanism(...)` pattern before adding the test.
- `tests/PreseasonWebcamAuto.java` is intentionally not a shared match op mode; keep it documented as validation/proof-of-concept code.
- `tests/MecanumTeleOp.java`, `tests/PreseasonWebcamAuto.java`, `tests/ftcSdk/RGBIndicatorTest.java`, and `tests/ftcSdk/SlidesMotorTest.java` were recently updated to centralize their hardware naming, so future test edits should preserve that alignment.

## Validation expectations

- Build after changing tuning or test classes.
- Prefer a robot-side sanity check before claiming tuning-related behavior is correct.
- If a test touches drive direction, check it with the robot safely off the ground first when practical.
- Use `MecanumTeleOp` when you need direct SDK-level drive and IMU behavior checks; use `MotorDirectionTest` when you want to validate the current `Constants` / `RobotConfigurations` drivetrain wiring at low speed.
- Use `tests/ftcSdk/MotorTest.java` when the goal is specifically to validate the `Motor` wrapper.
- Use `tests/ftcSdk/RGBIndicatorTest.java` and `tests/ftcSdk/SlidesMotorTest.java` when the goal is to validate custom FTC `HardwareDevice` integration directly.
- Use `pid/autoTune/PositionPIDTunerOpMode.java` for position mechanisms, `VelocityPIDFTunerOpMode.java` for single-motor velocity tuning, and `DualVelocityPIDFTunerOpMode.java` for matched dual-motor flywheels or similar paired velocity systems.
