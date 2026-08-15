---
name: tests-and-tuning-opmodes
description: Use this skill when asked to add or modify test op modes, Pedro tuning flows, FTC AutoTune op modes, or validation helpers under `tests/`, `pid/autoTune/`, and `pedroPathing/Tuning.java`.
---

# Tests and Tuning OpModes

Use this skill when work is focused on validation op modes, tuners, or measurement workflows instead of match-ready robot behavior.

## Upstream `~/.gradle` lookup

If a test or tuner behavior depends on an external library rather than TeamCode,
inspect `~/.gradle/caches/modules-2/files-2.1/` using the version pinned in
`build.dependencies.gradle`.

- `com.pedropathing/core` and `com.pedropathing/ftc` for Pedro tuning and
  follower behavior
- `com.pedropathing/telemetry` for `pedroPathing/Tuning.java`, telemetry, and
  op-mode selection issues
- `com.pedropathing/ivy` when a test flow depends on Ivy command timing
- `org.firstinspires.ftc/RobotCore`, `org.firstinspires.ftc/Hardware`, and
  `org.firstinspires.ftc/Vision` for FTC SDK tuner/test behavior

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `library/src/main/java/com/rmsftc/library/robot/RobotConfigurations.java`
- `library/src/main/java/com/rmsftc/library/math/Polynomial.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/FieldCentricMecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MotorDirectionTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PedroPathingTeleOpTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PlannerIntakeVisionAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/androidTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/*FactoryTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/IMUTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/MotorTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/RGBIndicatorTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/SlidesMotorTest.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/UnitTestAll.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/...`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/TuningConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/PositionPIDTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/VelocityPIDFTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/DualVelocityPIDFTunerOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/EquationCurve.java`
- `build.dependencies.gradle`

## Current repo patterns

- Test op modes live under `tests/` and are annotated for the driver station.
- `tests/ftcSdk/androidTest.java` is a shared `LinearOpMode` base for simple SDK-level validation flows that run one scripted check, sleep briefly, and then request stop. It now also provides `skipIfMechanismMissing(...)` and `showStep(...)` helpers so factory tests can stay profile-aware and telemetry-friendly.
- `tests/ftcSdk/` now includes factory-style runtime checks for `Follower`, `IMU`, `Intake`, `Limelight`, `RobotDrive`, `RGBIndicator`, and `LogitechWebcam`.
- `tests/ftcSdk/` now also includes custom-hardware checks for the device-level `RGBIndicator` and `SlidesMotor` classes under `library/hardware/hardwareDevice/`.
- TeamCode now also has a JVM test layer under `TeamCode/src/test/java/`, and the shared reusable-library tests now live under `library/src/test/java/com/rmsftc/library/...`; use those for math, utility, field/game, gamepad, robot-configuration, and `EquationCurve` behavior that does not need FTC runtime hardware.
- `pid/autoTune/` now contains FTC AutoTune relay-feedback tuners built around repo-local `pid.autoTune.master.PIDMaster` / `DualMotorPIDMaster` FTC adapters, the external `pidautotuner-core` algorithms, and a repo-local `@Configurable` `TuningConstants` class.
- Those FTC AutoTune op modes are driver-station-visible `@TeleOp` entries in group `FtcAutoTune`, and they are calibration tools rather than match-ready robot behavior.
- Pedro tuning is centralized in `pedroPathing/Tuning.java`, which exposes many selectable tuner sub-modes.
- Tuning flows rely on `Constants.createFollower(hardwareMap)` so they inherit the current robot profile and follower config.
- Other subsystems used in tests should also come from `Constants.create...(...)` so tests exercise the same construction paths as match code.
- Tests that call a `Constants.create...(...)` helper need the profile selected by `Constants.currentBot` to register that class first, or they should skip cleanly through `skipIfMechanismMissing(...)`.

## Existing examples

- `MecanumTeleOp` is a raw FTC iterative test op mode with robot-centric mode, field-centric mode, explicit IMU yaw reset, and a built-in motor-direction test mode. It initializes its IMU through `Constants.createIMU(hardwareMap)`.
- `FieldCentricMecanumTeleOp` also uses `Constants.createIMU(hardwareMap)`, so its IMU name and REV Hub orientation follow the active robot profile.
- `MotorDirectionTest` is a looping low-speed sanity check for the constants-driven drivetrain profile, and it also checks intake direction when the active profile registers `Intake.class`. It updates telemetry live while you flip directions.
- `PedroPathingTeleOpTest` mixes manual drive with on-demand path following for follower behavior checks.
- `TestPathPlanner` is the current planner-stack validation op mode; it builds a `PlanBuilder`, inserts a timed wait, and schedules the resulting `PathPlannerList`.
- `PlannerIntakeVisionAuto` is the current planner + Limelight wrapper validation op mode; it builds a `PlanBuilder`, follows a custom approach path, then exercises `intakeWVision(...)` with a fail-safe search path.
- `PreseasonWebcamAuto` is a proof-of-concept color-tracking `LinearOpMode` that uses a follower plus raw Limelight results.
- `MotorTest` is a small SDK-level wrapper check built on the custom `androidTest` base.
- `FollowerFactoryTest`, `RobotDriveFactoryTest`, `IntakeFactoryTest`, `LimelightFactoryTest`, `RGBFactoryTest`, and `WebcamFactoryTest` are short factory/construction smoke tests built on the shared `androidTest` base. `RobotDriveFactoryTest` validates the public `Constants.createDrive(...)` path and reports whether the active profile is using manual or follower drive.
- `IMUTest` is the runtime check for `Constants.createIMU(...)`; it resets yaw once, reports before/after values, and leaves live heading movement confirmation to the driver.
- `RGBIndicatorTest` validates both the device-level `library.hardware.hardwareDevice.RGBIndicator` and the profile-style `library.hardware.leds.RGBIndicator` wrapper, but its wrapper half is constructed with a temporary `new RobotConstants().addMechanism(...)` rather than `Constants.createRGB(...)`.
- `SlidesMotorTest` is a direct SDK-level validation flow for the custom `SlidesMotor` hardware device, and its hardware name now comes from `SlidesMotor.DEFAULT_HARDWAREMAP_NAME`.
- `UnitTestAll` is the single JVM suite entry point for non-hardware repo tests; it lives under TeamCode but now also imports the reusable-library tests for `DriveVector`, `Timer`, PIDF control, filters, field geometry, gamepad input, interpolation, symbolic math, and robot configuration.
- `PositionPIDTunerOpMode`, `VelocityPIDFTunerOpMode`, and `DualVelocityPIDFTunerOpMode` are thin wrappers around the FTC AutoTune library that run relay-feedback calibration plus optional `gamepad1.a` live tests.
- `TuningConstants` is the shared place for FTC AutoTune motor names, direction booleans, target RPM / ticks, relay amplitude, and feedforward sweep settings.
- `Tuning.java` contains localization, velocity, braking, PIDF, centripetal, and path-shape tuners.
- `EquationCurve` plus `Polynomial` are now part of the custom path-shape toolbox if a test or tuner needs non-Bezier geometry.
- `PreseasonWebcamAuto` also exercises the generic `PIDFCoefficients<Pose>` scalar-broadcast path through `setTolerance(0.5)` and `setIntegralLimit(1.0)`.

## Rules

- Keep validation op modes safe by using low power or explicit warnings when motors may move unexpectedly.
- Reuse the repo’s existing follower creation path so tuning matches real robot configuration.
- Reuse centralized subsystem factories from `Constants` so tests do not drift from production wiring.
- If a runtime factory test only proves construction, say that plainly in telemetry instead of implying full behavior coverage.
- Treat `pid/autoTune/` as an intentional exception to the normal subsystem-factory rule: those op modes pass raw motor names and directions through `TuningConstants` because the repo-local master adapters own the FTC relay-test integration.
- If a test intentionally bypasses the normal factory path, still source hardware names from `RobotConfigurations` or the owning nested `*_PARAMS` class instead of introducing standalone literals.
- Put one clear purpose per test op mode.
- When adding a tuner, explain what the driver should physically do and what value they should record.
- For FTC AutoTune op modes, document the two-phase flow (automatic relay test, optional live test) and call out any hard-stop or flywheel spin-up risks before the driver presses Start.
- If testing custom path geometry, make it explicit whether the path uses Bezier primitives or `EquationCurve` / `Polynomial` math.
- If a test intentionally bypasses the normal wrapper flow, document that it is a proof-of-concept or SDK-level validation mode.

## Important repo notes

- The selected robot profile may register only part of the factory surface. Runtime tests for optional mechanisms should skip with clear telemetry instead of failing on `Constants.assertHasMechanism(...)`.
- Real robot measurements matter more than code-level plausibility here.
- `build.dependencies.gradle` now resolves JitPack plus `com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.3.7`; the removed FTC artifact is replaced by repo-local master adapters, so tuning/build work should keep both layers aligned.
- Some tuner code comes from Pedro tooling and is intentionally verbose; keep repo-specific adjustments minimal and clear.
- If a new mechanism is being validated, make sure it follows the same `Mechanism` + `RobotConstants.createMechanism(...)` pattern before adding the test.
- `tests/PreseasonWebcamAuto.java` is intentionally not a shared match op mode; keep it documented as validation/proof-of-concept code.
- `tests/PreseasonWebcamAuto.java` and `tests/ftcSdk/RGBIndicatorTest.java` still intentionally bypass the normal factory path for parts of their behavior. Check registration in the profile selected by `Constants.currentBot` before adding a factory-created wrapper to either test.
- `tests/MecanumTeleOp.java`, `tests/PreseasonWebcamAuto.java`, `tests/ftcSdk/RGBIndicatorTest.java`, and `tests/ftcSdk/SlidesMotorTest.java` were recently updated to centralize their hardware naming, so future test edits should preserve that alignment.

## Validation expectations

- Build after changing tuning or test classes.
- Run `:TeamCode:testDebugUnitTest` after changing JVM-test-covered library or configuration code, and use `UnitTestAll` when you want one explicit suite entry point.
- Prefer a robot-side sanity check before claiming tuning-related behavior is correct.
- If a test touches drive direction, check it with the robot safely off the ground first when practical.
- Use `MecanumTeleOp` when you need direct SDK-level drive and IMU behavior checks; use `MotorDirectionTest` only when the active profile includes `Intake.class` and you want to validate the current `Constants` / `RobotConfigurations` drivetrain plus intake-direction wiring at low speed.
- Use `tests/ftcSdk/MotorTest.java` when the goal is specifically to validate the `Motor` wrapper.
- Use `tests/ftcSdk/RGBIndicatorTest.java` and `tests/ftcSdk/SlidesMotorTest.java` when the goal is to validate custom FTC `HardwareDevice` integration directly.
- Use the `tests/ftcSdk/*FactoryTest.java` and `IMUTest.java` op modes when the goal is to validate the `Constants` / `RobotConstants.createMechanism(...)` construction path on hardware without writing a new custom test flow.
- Use `pid/autoTune/PositionPIDTunerOpMode.java` for position mechanisms, `VelocityPIDFTunerOpMode.java` for single-motor velocity tuning, and `DualVelocityPIDFTunerOpMode.java` for matched dual-motor flywheels or similar paired velocity systems.
