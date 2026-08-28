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
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/VisionFactoryTest.java`

## Existing abstraction

- `Vision` is the profile-managed camera mechanism and is created with
  `Constants.createVision(hardwareMap)` or obtained from `Robot`.
- `Vision.VISION_PARAMS` owns the nested `Limelight.LIMELIGHT_PARAMS`, so
  hardware names, geometry, polling, and timeout settings remain configurable
  per profile.
- `Vision.Pipeline` exposes `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR`.
- `start(...)`, `stop()`, `setPipeline(...)`, `updateRobotOrientation(...)`,
  and cached result access all go through `Vision`; `Limelight` is private to
  that mechanism.
- `Vision.periodic(...)` refreshes cached results. Direct owners that do not
  call aggregate periodic must call `Vision.update()` regularly.
- Periodic updates do not apply MT2 pose data to the drive. Call
  `Robot.forceRelocalize()` or `RobotIvyBuilder.forceRelocalize()` explicitly
  when a valid, running Vision pipeline should replace the drive pose.

## Repo conventions

- The shared teleops create the `Robot` aggregate, retrieve `Vision`, and
  start it in `APRIL_TAG` mode.
- `tests/PreseasonWebcamAuto` starts the camera in `COLOR` mode as a proof-of-concept validation flow.
- `tests/ftcSdk/VisionFactoryTest` is the current runtime smoke test for
  profile-managed Vision creation and repeated updates on the AprilTag pipeline.
- `library/planners/paths/Utils.IntakeWVision` is the current in-repo wrapper consumer that builds a vision-assisted path from detector results and falls back to a fail-safe path if detection/path generation does not succeed in time.
- `tests/PlannerIntakeVisionAuto.java` is the driver-station-visible
  integration test for that planner step. It requires Vision, starts it on
  `NEURAL_DETECTOR`, schedules the plan, and stops it during cleanup.
- Shared Limelight behavior must use Vision's cached result API. The deprecated
  `pedroPathing/custom/LimelightPinpointLocalizer.java` is the remaining raw SDK
  exception.
- Vision creation depends on profile registration; runtime tests should use
  `Constants.createVision(...)` or obtain it from `Robot`.

## Params and creation pattern

When working on the Limelight integration:

1. keep wrapper-specific helper params with the wrapper class and builder-style
2. keep `Limelight` private to the `Vision` mechanism
3. register `Vision` with each applicable robot profile
4. expose construction through `Constants.createVision(...)`
5. let the owning flow choose Vision's pipeline and start/stop lifecycle

The important repo-level uniformity point is the shared `Mechanism` + `RobotConstants.createMechanism(...)` construction path.

## Rules

- Keep pipeline selection explicit in the op mode or planner that owns the behavior.
- Extend `Vision` when new shared camera behavior is needed rather than scattering `Limelight3A` calls across op modes.
- Tolerate missing or invalid results; `Vision.update()` handles absent data as a normal runtime condition.
- `Limelight.getPipelineIndex()` is not available until the device has
  produced a result; it throws instead of returning a fabricated pipeline.
- Keep coordinate-system conversion explicit when wiring pose/orientation data to Pedro consumers; the current repo path is `Constants.FTC_COORDINATE_SYSTEM` plus `MathUtils.units`.
- If new vision tuning fields are added, keep them in `Vision.VISION_PARAMS`
  and its nested `LIMELIGHT_PARAMS` rather than scattering them through op modes.

## Current limitations to remember

- The distance helper uses placeholder mounting constants, so those values are not trustworthy until measured.
- `isDisconnected` depends on repeated updates and the configured timeout.
- Vision fails at construction when the configured FTC hardware name does not
  resolve; this makes camera ownership and configuration explicit.

## Good extension points

- Add helper methods that interpret fiducial, color, or detector results.
- Add telemetry-friendly summary methods.
- Add pipeline-specific convenience behavior while keeping pipeline numbers hidden behind the enum.
- If adding a new camera wrapper class, mirror the same `Mechanism` + `(HardwareMap, RobotConstants)` + `Constants.create...(...)` pattern used here.
