---
name: limelight-wrapper-and-pipelines
description: Use this skill when asked to work directly on the TeamCode Limelight wrapper, switch pipelines, or consume color, AprilTag, and detector results through the existing abstraction.
---

# Limelight Wrapper and Pipelines

Use this skill for Limelight-specific work that is narrower than the broader localization-and-vision skill.

## Upstream `~/.gradle` lookup

If a pipeline or camera behavior appears to come from an SDK dependency instead
of the TeamCode wrapper, inspect `~/.gradle/caches/modules-2/files-2.1/` using
the version pinned in `build.dependencies.gradle`.

- `org.firstinspires.ftc/Vision` for FTC camera, processor, and vision-portal
  behavior
- `org.firstinspires.ftc/RobotCore` and `org.firstinspires.ftc/Hardware` for
  supporting SDK details
- `com.pedropathing/ftc` when the Limelight issue interacts with Pedro
  localization/pathing code

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/MathUtils.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PlannerIntakeVisionAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/LimelightFactoryTest.java`

## Existing abstraction

- `Constants.createLimelight(hardwareMap)` is the normal factory entry point when the active robot profile registers `Limelight.class`.
- `Limelight` extends `Mechanism`, so the normal factory path is `Constants.createLimelight(...)` → `currentBot.createMechanism(...)`.
- `Limelight.Pipeline` currently exposes `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR`.
- `start(...)` chooses a pipeline and starts the FTC `Limelight3A` device.
- `LLUpdate()` pulls the latest result, tracks disconnect state, and caches fiducials, colors, and detectors.
- `getDetectorHighestConfidence()` is the current wrapper helper for picking one detector result from the cached neural-detector list.
- `updateRobotOrientation(double orientationRad)` now expects a Pedro heading in radians and converts it into the FTC/Limelight frame through `Constants.FTC_COORDINATE_SYSTEM.convertFromPedro(...)`.
- `getBotposeMT2()` now returns a Pedro `Pose`, converting Limelight MT2 `Pose3D` data through `MathUtils.units.toPedroPathing(...)`.
- `periodic(GpBindings GpBindings)` is the mechanism-wide update hook, and `updateCommand(GpBindings GpBindings)` is the Limelight-specific Ivy convenience wrapper around that call.

## Repo conventions

- The shared teleops create a `Mechanisms` aggregate, then read `Limelight` through `mechanisms.get(Limelight.class)` and start it in `APRIL_TAG` mode only when that optional mechanism is present.
- `tests/PreseasonWebcamAuto` starts the camera in `COLOR` mode as a proof-of-concept validation flow.
- `tests/ftcSdk/LimelightFactoryTest` is the current runtime smoke test for wrapper creation plus repeated `LLUpdate()` calls on the AprilTag pipeline.
- `library/planners/paths/Utils.IntakeWVision` is the current in-repo wrapper consumer that builds a vision-assisted path from detector results and falls back to a fail-safe path if detection/path generation does not succeed in time.
- `tests/PlannerIntakeVisionAuto.java` is the driver-station-visible integration test for that planner step. It starts the optional wrapper on `NEURAL_DETECTOR`, schedules the plan, and stops the wrapper during cleanup.
- Even though `PreseasonWebcamAuto` intentionally uses raw `Limelight3A` calls, it now sources the hardware name from `Limelight.LIMELIGHT_PARAMS` instead of a test-only string literal.
- Shared Limelight behavior should read from the wrapper’s cached fields; the current raw-SDK exceptions are `tests/PreseasonWebcamAuto.java` and the deprecated `pedroPathing/custom/LimelightPinpointLocalizer.java`.
- Wrapper-based Limelight creation through `Constants.createLimelight(...)`
  requires the selected robot profile to register `Limelight.class`; runtime
  tests should guard optional availability with `Constants.hasMechanism(...)` or
  `skipIfMechanismMissing(...)` instead of assuming every helper is active.

## Params and creation pattern

When working on `Limelight` or adding a similar wrapper:

1. keep wrapper-specific helper params with the wrapper class when they belong there, and keep them builder-style when the wrapper already follows the repo’s params pattern
2. make the wrapper extend `Mechanism` if it should participate in the standard robot-subsystem flow
3. keep the constructor shape `(HardwareMap, RobotConstants)` so `RobotConstants.createMechanism(...)` can instantiate it
4. register the wrapper in the active robot profile with `.addMechanism(...)`
5. expose construction through `Constants.create...(...)`
6. avoid direct wrapper construction in op modes unless the task is specifically refactoring the factory surface

The important repo-level uniformity point is the shared `Mechanism` + `RobotConstants.createMechanism(...)` construction path.

## Rules

- Call `LLUpdate()` regularly if the Limelight is active.
- Keep pipeline selection explicit in the op mode or mechanism that owns the behavior.
- Extend the wrapper when new camera behavior is needed rather than scattering `Limelight3A` calls across op modes.
- Tolerate missing or invalid results; `LLUpdate()` already handles absent data as a normal runtime condition.
- Keep coordinate-system conversion explicit when wiring pose/orientation data to Pedro consumers; the current repo path is `Constants.FTC_COORDINATE_SYSTEM` plus `MathUtils.units`.
- If new vision tuning fields are added, keep them on `LIMELIGHT_PARAMS` or the wrapper itself rather than scattering them through op modes.
- If the wrapper can be disabled by profile selection, make sure it decides that before touching hardware.

## Current limitations to remember

- The distance helper uses placeholder mounting constants, so those values are not trustworthy until measured.
- `isDisconnected` depends on repeated updates and the internal timer.
- The wrapper’s no-device mode is profile-driven: omitting `Limelight.class` from the active `RobotConstants.Mechanisms` set makes the constructor return before any hardware lookup.

## Good extension points

- Add helper methods that interpret fiducial, color, or detector results.
- Add telemetry-friendly summary methods.
- Add pipeline-specific convenience behavior while keeping pipeline numbers hidden behind the enum.
- If adding a new camera wrapper class, mirror the same `Mechanism` + `(HardwareMap, RobotConstants)` + `Constants.create...(...)` pattern used here.
