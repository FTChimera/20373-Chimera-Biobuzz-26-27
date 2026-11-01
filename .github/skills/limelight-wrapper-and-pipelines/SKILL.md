---
name: limelight-wrapper-and-pipelines
description: Use this skill for the profile-managed Limelight/Vision mechanism and its pipelines.
---

# Limelight Wrapper and Pipelines

`mechanisms/Vision.java` is the current profile-managed camera interface. It
extends `Mechanism`, owns a single lower-level
`library/hardware/webcam/Limelight` controller instance, and exposes a
`Vision.Pipeline` enum (`APRIL_TAG`, `COLOR`, `NEURAL_DETECTOR`) that wraps the
underlying `Limelight.Pipeline` (indices `0`, `1`, `2`). `Limelight` itself is a
standalone controller over a `Limelight3A` device; it is not a `Mechanism` and
can be constructed directly from a `HardwareMap` or an existing `Limelight3A`
if a test needs to bypass profile registration.

## Params and construction

`Vision.VISION_PARAMS` (a `Mechanism.Params<VISION_PARAMS>`) carries a nested
`Limelight.LIMELIGHT_PARAMS` (mount angle degrees, lens height inches, tag
height inches, X/Y camera offsets, hardware map name defaulting to
`"limelight"`, poll rate Hz, and a disconnect timeout in seconds) plus an
`AUTO_INTAKE_BUTTON` (`Button.DPAD_DOWN` by default) consumed during teleop
periodic. `Vision(HardwareMap, RobotConstants)` resolves these params through
`resolveParams(Vision.class, new VISION_PARAMS())` and constructs the
`Limelight` from them. Register `Vision.class` in `RobotConfigurations` (see
`RobotConfigurations.FirstRobot()`), then create it through
`Constants.createVision(...)` or retrieve it from `Robot.getVision()` /
`Robot.requireVision()`.

## Lifecycle

Let `Robot` own the lifecycle: `Robot.start()` always calls
`startVision(Vision.Pipeline.APRIL_TAG)` if Vision is registered, and
`Robot.stop(...)` calls every mechanism's `stop()`, which for `Vision` calls
`camera.stop()`. `tele/TeleOp.runOpMode()` also calls
`robot.startVision(Vision.Pipeline.APRIL_TAG)` explicitly after `waitForStart()`.
Avoid starting/stopping the camera directly from op mode code when the
aggregate `Robot` is in use; call `robot.startVision(...)` /
`robot.stopVision()` / `robot.setVisionPipeline(...)` instead so the same
`Limelight` instance and cached results are shared with `RobotDrive` and
`forceRelocalize()`.

## Periodic behavior and teleop intake button

`Vision.periodic(State)` only calls `updateRobotOrientation(heading)` then
`update()` to refresh cached Limelight results; it does **not** relocalize the
drive pose (that line is commented out in source). When `state.getRobotMode()`
is `TELEOP` and `state.getGpBindings()` is non-null, periodic reads
`params.AUTO_INTAKE_BUTTON` and, on `wasJustPressed()`, calls the **no-argument**
`IntakeBallPath.generateIntakePathFromCam()` and follows the result via
`state.getRobot().getDrive().followPath(...)`. That no-argument overload
currently returns `new PathChain()` (empty), so the teleop button is a wired
scaffold, not a working auto-intake trigger yet; the real generator is the
5-argument overload described below.

## Results and pipeline selection

Use the typed `getFiducials()`, `getColors()`, `getDetectors()`, or
`getDetectorHighestConfidence()` accessors (all backed by
`LLResultTypes.*Result` lists from the last `update()`). `hasValidResult()`
checks `camera.result != null && camera.result.isValid()`; `isDisconnected()`
reflects the `LIMELIGHT_PARAMS.DISCONNECT_TIMEOUT_SECONDS` watchdog inside
`Limelight.update()`. Results can be missing, stale, or belong to whichever
pipeline is currently active — callers must not assume a specific pipeline is
selected just because a result object exists.

## Explicit relocalization (`forceRelocalize`)

Vision never automatically writes the drive pose. `Vision.forceRelocalize(RobotDrive drive)`
(called through `Robot.forceRelocalize()`) is the deliberate blend path:

1. Returns `false` immediately if the camera `isRunning()` is false or there is
   no valid result.
2. Remembers the current pipeline index, force-switches to `APRIL_TAG`,
   updates robot orientation from `drive.getPose().getHeading()`, and calls
   `update()`.
3. Reads `getCachedPose()` (delegates to `getBotposeMT2()`), then blends it with
   the current drive pose using `Pose.linearCombination(...)` with a fixed
   `CAMERA_CONFIDENCE = 0.3` weight on the vision pose and `0.7` on the existing
   drive pose (a simple low-pass filter), and calls `drive.setPose(...)`.
4. Restores the previously active pipeline before returning `true`.

Treat this as a one-shot correction to call when a valid AprilTag read is
available, not a substitute for periodic localization.

## Limelight controller internals

`Limelight.updateRobotOrientation(orientationRad)` converts the Pedro heading
into the FTC/Limelight frame via
`Constants.FTC_COORDINATE_SYSTEM.convertFromPedro(new Pose(0, 0, orientationRad))`
before calling the SDK's `updateRobotOrientation`. `calculateDistance(fiducial)`
uses `(TAG_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / tan(mountAngle + ty)`.
`getBotposeMT2()` returns `null` unless the last result is valid and non-null,
otherwise converts through `MathUtils.units.toPedroPathing(botpose)`.

## Vision-assisted intake paths

`IntakeBallPath.generateIntakePathFromCam(follower, robotPose, vision, numBalls,
detectors)` is the robust detector consumer used by autonomous/planner code. It
filters candidates by `MIN_CONFIDENCE` (0.35) and `MIN_TARGET_AREA` (0.15),
estimates forward distance from target-Y geometry with an area-based fallback
(`AREA_DISTANCE_SCALE / sqrt(area)`), rejects candidates outside
`MIN_FORWARD_DISTANCE_INCHES`/`MAX_FORWARD_DISTANCE_INCHES`/
`MAX_LATERAL_OFFSET_INCHES`, deduplicates within `MERGE_DISTANCE_INCHES`, then
picks an ordered forward sequence with a dynamic-programming search that
penalizes turning (`SCORE_TURN_WEIGHT`) and gaps (`SCORE_GAP_WEIGHT`) before
building either a Pedro `curveThrough` chain or an adaptive-handle Bezier
chain. It may legitimately return `null`; callers must keep a fallback plan.

`LogitechWebcam` and `Constants.createWebcam(...)` are deprecated compatibility
surfaces annotated `@Deprecated`; new code must use `Vision`.
