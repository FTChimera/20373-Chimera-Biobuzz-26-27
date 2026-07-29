---
name: localization-and-vision
description: Use this skill when asked to work on Limelight, AprilTag pose updates, Pinpoint localization, color pipelines, or vision-assisted autonomous behavior in this repo.
---

# Localization and Vision for This Robot Code

This repository currently mixes Pinpoint-based Pedro localization with two separate vision wrappers: a Limelight wrapper and a VisionPortal AprilTag webcam wrapper.

## Relevant files

- `build.dependencies.gradle`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/LogitechWebcam.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`

## Verified current state

1. The build includes FTC Vision `11.1.0` plus Pedro Pathing / Ivy dependencies.
2. `library/hardware/webcam/Limelight.java` extends `Mechanism`, wraps `Limelight3A`, supports `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR` pipelines, and exposes `updateCommand()` as `Commands.infinite(Limelight.this::LLUpdate)`.
3. `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` both create a Limelight through `Constants.createLimelight(...)`, start it in `APRIL_TAG` mode, and refresh it every loop.
4. The `OldDECODERobotTesting` and `PreseasonDrivetrain` profiles exist in `RobotConfigurations`. Verify the profile selected by `Constants.currentBot` registers `Limelight` or `LogitechWebcam` before calling its corresponding `Constants.create...(...)` helper.
5. `tests/PreseasonWebcamAuto.java` bypasses the wrapper, reads raw `Limelight3A` color results, switches pipeline `1`, and gets the hardware name from `new Limelight.LIMELIGHT_PARAMS().HARDWAREMAP_NAME`.
6. `library/hardware/webcam/LogitechWebcam.java` is a separate `Mechanism` wrapper around `VisionPortal` + `AprilTagProcessor`. `Constants.createWebcam(...)` exists, but no current robot profile registers `LogitechWebcam` and no TeamCode op mode instantiates it.
7. `pedroPathing/custom/LimelightPinpointLocalizer.java` is `@Deprecated` and is not the default follower-localizer path.

## Wrapper configuration pattern that exists in code

Vision wrappers currently follow the same basic construction pattern as other mechanisms:

- extend `Mechanism`
- keep the constructor shape `(HardwareMap, RobotConstants)`
- gate hardware access with a profile-aware params object before calling `hardwareMap.get(...)`
- register the wrapper in the chosen robot profile with `.addMechanism(...)`
- expose construction through `Constants.create...(...)` when higher-level code should call it

The active selected profile currently registers the Limelight wrapper but not the Logitech webcam wrapper, so webcam-based vision work through `Constants` is still a profile-selection task as well as a wrapper/op-mode task.

## Repo-specific constraints

- `Limelight.LLUpdate()` must be called regularly to keep cached results and disconnected-state tracking meaningful.
- Vision results can be missing or invalid; current code already checks for null/invalid Limelight results and empty AprilTag detections.
- `Limelight.calculateDistance(...)` uses mount-angle, lens-height, and tag-height params that default to `0`, so those distance values are not trustworthy until real measurements are filled in.
- `LogitechWebcam.update()` uses the first current AprilTag detection, converts FTC pose data into a `Pose3D`, then maps `x`, `z`, and yaw into `pedroPose`.
- `LimelightPinpointLocalizer` updates Pinpoint first and then tries to override pose from Limelight MT2 botpose, so any fusion changes must keep coordinate conversions explicit.

## Rules

- Keep pipeline choice explicit in the op mode or wrapper start path.
- If a task needs color detection, follow the current repo pattern: keep the follower on Pinpoint and use Limelight results separately unless you are intentionally redesigning fusion.
- Do not introduce silent pose overwrites without checking for invalid or missing vision data.
- If a new camera wrapper should be part of the normal robot architecture, make it fit the `Mechanism` + `RobotConstants.createMechanism(...)` + `Constants.create...(...)` flow.
- If a proof-of-concept path intentionally bypasses the wrapper, still keep hardware naming centralized through the wrapper params or selected robot profile.

## Example decisions

- Need AprilTag botpose from Limelight without changing default localization: extend `Limelight` and keep the follower on `Constants.createFollower(...)`.
- Need color detection in a test or proof-of-concept op mode: copy the `PreseasonWebcamAuto` pattern of separate Limelight color reads plus normal follower updates.
- Need VisionPortal AprilTag detections: use `LogitechWebcam`, but also update the selected robot profile by calling `.addMechanism(LogitechWebcam.class, ...)` because neither current profile registers it.
- Need vision-assisted localization fusion: start by auditing `LimelightPinpointLocalizer.java` and `RobotConstants.createFollower(...)` together, because the fusion helper is not wired into the default follower path.
