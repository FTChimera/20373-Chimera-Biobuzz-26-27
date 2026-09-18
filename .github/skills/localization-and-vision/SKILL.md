---
name: localization-and-vision
description: Use this skill when asked to work on Limelight, AprilTag pose updates, Pinpoint localization, color pipelines, or vision-assisted autonomous behavior in this repo.
---

# Localization and Vision for This Robot Code

This repository currently mixes Pinpoint-based Pedro localization with two separate vision wrappers: a Limelight wrapper and a VisionPortal AprilTag webcam wrapper.

## Upstream `~/.gradle` lookup

If localization or vision behavior depends on external code rather than
TeamCode, inspect `~/.gradle/caches/modules-2/files-2.1/` using the versions
from `build.dependencies.gradle`.

- `com.pedropathing/core` and `com.pedropathing/ftc` for Pedro pose, pathing,
  and follower behavior
- `org.firstinspires.ftc/Vision` for AprilTag, VisionPortal, and related FTC
  vision APIs
- `org.firstinspires.ftc/RobotCore` and `org.firstinspires.ftc/Hardware` for
  supporting SDK details

## Relevant files

- `build.dependencies.gradle`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/math/MathUtils.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/LogitechWebcam.java` (deprecated)
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PlannerIntakeVisionAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/ftcSdk/VisionFactoryTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`

## Verified current state

1. The build includes FTC Vision `11.2.1` plus Pedro Pathing / Ivy dependencies.
2. `mechanisms/Vision.java` extends `Mechanism`, owns the standalone
   `Limelight` controller, supports `APRIL_TAG`, `COLOR`, and
   `NEURAL_DETECTOR` pipelines, and updates through
   `periodic(State, RobotMode)`.
3. `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` create teleop `Robot`
   aggregates, retrieve `Vision`, start its `APRIL_TAG` pipeline, and refresh
   it through aggregate periodic updates.
4. Vision wrapper availability is profile-driven. Verify the selected profile
   registers `Vision` before calling its corresponding
   `Constants.create...(...)` helper.
5. `tests/PreseasonWebcamAuto.java` is a proof-of-concept flow that creates
   `Vision` directly through `Constants.createVision(...)`, selects `COLOR`,
   updates Vision, and uses its color results beside a normal follower loop.
6. `Limelight.updateRobotOrientation(...)` now takes a Pedro heading in radians and converts it into the FTC/Limelight frame through `Constants.FTC_COORDINATE_SYSTEM`, while `getBotposeMT2()` converts Limelight MT2 `Pose3D` results back into Pedro `Pose` through `MathUtils.units.toPedroPathing(...)`.
7. `library/hardware/webcam/LogitechWebcam.java` is a deprecated standalone `VisionPortal` + `AprilTagProcessor` wrapper. `Constants.createWebcam(...)` remains only for compatibility; new code should use the profile-managed `Vision` mechanism.
8. `tests/ftcSdk/VisionFactoryTest.java` is the current driver-station smoke
   test for Vision mechanism creation and AprilTag polling.
9. `library/planners/paths/Utils.IntakeWVision` is the current vision-assisted pathing example: it reads detector results from the wrapper, preserves a fail-safe path, and can choose between Pedro curve-through and adaptive-handle Bezier generation.
10. `pedroPathing/custom/LimelightPinpointLocalizer.java` is `@Deprecated` and is not the default follower-localizer path.
11. `tests/PlannerIntakeVisionAuto.java` is the driver-station-visible
    integration test for `IntakeWVision`; it requires Vision, uses the
    neural-detector pipeline for planning, and stops it during cleanup.

## Wrapper configuration pattern that exists in code

Vision wrappers currently follow the same basic construction pattern as other mechanisms:

- extend `Mechanism`
- keep the constructor shape `(HardwareMap, RobotConstants)`
- gate hardware access with a profile-aware params object before calling `hardwareMap.get(...)`
- register the wrapper in the chosen robot profile with `.addMechanism(...)`
- expose construction through `Constants.create...(...)` when higher-level code should call it

Vision work through `Constants` is therefore a profile-registration task as
well as a wrapper/op-mode task: some profiles may expose neither camera wrapper
even though the helper methods exist.

## Repo-specific constraints

- `Vision.update()` must be called regularly to keep cached results and disconnected-state tracking meaningful.
- Vision results can be missing or invalid; current code already checks for null/invalid Limelight results and empty AprilTag detections.
- `Limelight.calculateDistance(...)` uses mount-angle, lens-height, and tag-height params that default to `0`, so those distance values are not trustworthy until real measurements are filled in.
- The deprecated `LogitechWebcam.update()` uses the first current AprilTag detection, converts FTC pose data into a `Pose3D`, then maps `x`, `z`, and yaw into `pedroPose`.
- `LimelightPinpointLocalizer` updates Pinpoint first and then tries to override pose from Limelight MT2 botpose, so any fusion changes must keep coordinate conversions explicit.
- `Vision.periodic(...)` only updates orientation/results. It never
  automatically overwrites the drive pose; use `Vision.forceRelocalize(drive)`
  or the Robot / RobotIvyBuilder convenience method deliberately.

## Rules

- Keep pipeline choice explicit in the op mode or wrapper start path.
- If a task needs color detection, follow the current repo pattern: keep the follower on Pinpoint and use Limelight results separately unless you are intentionally redesigning fusion.
- Do not introduce silent pose overwrites without checking for invalid or missing vision data.
- If a new camera wrapper should be part of the normal robot architecture, make it fit the `Mechanism` + `RobotConstants.createMechanism(...)` + `Constants.create...(...)` flow.
- If a proof-of-concept path intentionally bypasses the wrapper, still keep hardware naming centralized through the wrapper params or selected robot profile.

## Example decisions

- Need AprilTag botpose from Limelight without changing default localization: extend `Limelight` and keep the follower on `Constants.createFollower(...)`.
- Need color detection in a test or proof-of-concept op mode: copy the `PreseasonWebcamAuto` pattern of separate Limelight color reads plus normal follower updates.
- Need AprilTag detections in new code: use the profile-managed `Vision` mechanism.
- Need vision-assisted localization fusion: start by auditing `LimelightPinpointLocalizer.java` and `RobotConstants.createFollower(...)` together, because the fusion helper is not wired into the default follower path.
