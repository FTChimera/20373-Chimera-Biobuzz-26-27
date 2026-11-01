---
name: tests-and-tuning-opmodes
description: Use this skill for TeamCode validation modes, Pedro tuning, and FTC AutoTune.
---

# Tests and Tuning Op Modes

## Runtime validation base class

`tests/ftcSdk/androidTest.java` is the shared abstract `LinearOpMode`:
subclasses implement `test(HardwareMap, GamepadEx gamepad1, GamepadEx
gamepad2, Telemetry)` and `getTestName()`. `runOpMode()` wraps both gamepads in
a `DualGamepad`, telemetries the test name, `waitForStart()`, updates the
gamepads once, calls `test(...)`, sleeps 2 seconds, then
`requestOpModeStop()`. It also exposes `skipIfMechanismMissing(Class,
Telemetry)` (checks `Constants.hasMechanism(...)`, telemetries the configured
mechanism set and returns `true` if the requested mechanism is absent so a
test can skip cleanly) and `showStep(Telemetry, String)` for step-by-step
telemetry.

Current `tests/ftcSdk/*` factory/device tests extending `androidTest`:
`FollowerFactoryTest`, `IMUTest`, `IntakeFactoryTest` (uses
`skipIfMechanismMissing(Intake.class, ...)`), `MotorTest`, `RGBIndicatorTest`,
`RobotDriveFactoryTest`, `SlidesMotorTest`, `VisionFactoryTest`, and
`WebcamFactoryTest`.

## Other current runtime surfaces

- `tests/MecanumTeleOp.java` and `tests/MTeleWIntakeTest.java` (plain `OpMode`s)
  for direct mecanum/IMU/intake inspection, reading motor names straight off
  `Constants.currentBot` and skipping intake logic when
  `Constants.hasMechanism(Intake.class)` is false.
- `tests/PedroPathingTeleOpTest.java` for follower teleop/path transition
  checks; `tests/FieldCentricMecanumTeleOp.java`, `tests/LLTester.java`,
  `tests/MotorDirectionTest.java`, and `tests/PreseasonWebcamAuto.java` remain
  as standalone direct-hardware validation op modes outside the profile
  factory path.
- `pedroPathing/Tuning.java` is a large `SelectableOpMode`-based menu (Pedro's
  `com.pedropathing.telemetry.SelectableOpMode`) that wires FTC Dashboard
  (`FtcDashboard`, `MultipleTelemetry`) drawing/canvas helpers for Pedro's
  standard tuning suite.
- `pid/autoTune/*` provides relay-feedback tuners:
  `PositionPIDTunerOpMode` (`@TeleOp(name = "Position PIDF Tuner", group =
  "FtcAutoTune")`), `VelocityPIDFTunerOpMode` (`"Velocity PIDF Tuner"`), and
  `DualVelocityPIDFTunerOpMode` (`"Dual Velocity PIDF Tuner"`), backed by
  `pid/autoTune/master/PIDMaster` and `DualMotorPIDMaster`.

`TestPathPlanner` and `PlannerIntakeVisionAuto` were deleted from source. Use
`autos/Garden8` plus focused planner/unit validation for current planner work.

## `TuningConstants` (`pid/autoTune/TuningConstants.java`)

Central, mutable static config for the AutoTune op modes: `MOTOR_NAME`/
`MOTOR_NAME_2`, `REVERSED`/`REVERSED_2`, `DUAL_ENCODERS`, relay test knobs
(`RELAY_AMPLITUDE`, `CYCLES_TO_COLLECT`, `CYCLES_TO_IGNORE`,
`RELAY_TEST_TIMEOUT_S`), position-tuner knobs (`POSITION_TARGET_TICKS`,
`POSITION_HYSTERESIS_TICKS`, `POSITION_LIVE_TEST_GAIN`), and velocity-tuner
knobs (`TICKS_PER_REV`, `TARGET_RPM`, `VELOCITY_HYSTERESIS_RPM`,
`FEEDFORWARD_TEST_POWERS`, `FEEDFORWARD_SETTLE_TIME_S`,
`TUNE_INTEGRAL_TERM`, `VELOCITY_LIVE_TEST_GAIN`), plus conversion helpers
`targetTicksPerSec()`, `hysteresisTicksPerSec()`, and `toRPM(double)`. It is
intentionally outside the profile mechanism factory and is edited directly
per-mechanism before running a tuner op mode.

## Host-side JVM tests

`TeamCode/src/test/java/.../UnitTestAll.java` is a JUnit `@Suite` listing every
concrete test class to run together:
`BlackboardTest`, `DriveVectorTest`, `TimerTest`, `IntakeBallPathTest`,
`BangBangControllerTest`, `CompositeControllerTest`, `PIDFControlTest`,
`LowPassFilterTest`, `FieldTest`, `StateTest`, `GamepadInputTest`,
`InterpolationTest`, `EquationMathTest`, `MathUtilsTest`,
`RobotConfigurationTest`, and `EquationCurveTest`. Add a new JVM test class
here (or extend an existing one) for any algorithmic change — e.g.
`IntakeBallPath` candidate scoring/selection, `MathUtils`, composite/bang-bang
controllers, dual-gamepad input, `State`, or profile configuration — before
relying on on-robot verification.

## Safety

Factory success proves construction, not safe mechanical behavior. Use low
power, wheels clear of the ground, clear telemetry, and direct driver
instructions for any test that can move a mechanism. AutoTune reads its own
`TuningConstants` independently of the profile mechanism factory, so verify
`MOTOR_NAME`/`REVERSED` match the physical setup before a relay test spins
anything up.

## AutoTune master classes

`pid/autoTune/master/PIDMaster` drives the single-motor relay-feedback flow
shared by `PositionPIDTunerOpMode` and `VelocityPIDFTunerOpMode`: it runs the
relay oscillation, computes Ziegler-Nichols-family gain candidates (including
the "classic ZN" and "no overshoot" rows referenced by
`TuningConstants.POSITION_LIVE_TEST_GAIN` / `VELOCITY_LIVE_TEST_GAIN`), and
supports holding gamepad A to live-test a candidate. `DualMotorPIDMaster` is
the two-motor variant behind `DualVelocityPIDFTunerOpMode`, adding a second
`DcMotorEx` and an optional second encoder controlled by
`TuningConstants.DUAL_ENCODERS`. Both master classes read directly from
`TuningConstants` rather than from any `RobotConstants` profile, so tuning one
mechanism never risks mutating match configuration.

## When to add vs. reuse a test

Prefer extending `androidTest` and adding a new `tests/ftcSdk/*FactoryTest`
when validating a new mechanism factory path (mirrors the existing
`*FactoryTest` naming and `skipIfMechanismMissing(...)` pattern). Prefer a JVM
test under `UnitTestAll` for anything that can run without hardware — pure
math, scoring/selection logic, or data-class behavior. Reserve a standalone
`tests/*.java` `OpMode`/`LinearOpMode` (like `MecanumTeleOp` or `LLTester`) for
direct-hardware debugging that intentionally bypasses the profile factory.
