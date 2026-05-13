---
name: localization-and-vision
description: Use this skill when asked to work on Limelight, AprilTag pose updates, Pinpoint localization, color pipelines, or vision-assisted autonomous behavior in this repo.
---

# Localization and Vision for This Robot Code

This repository currently mixes GoBILDA Pinpoint odometry with Limelight usage, depending on the mode.

## Relevant files

- `Constants.java`
- `library/hardware/webcam/Limelight.java`
- `pedroPathing/custom/LimelightPinpointLocalizer.java`
- `opModes/PreseasonWebcamAuto.java`
- `opModes/IvyTeleOp.java`
- `opModes/TeleOp.java`

## Current usage patterns

1. `Limelight` wraps `Limelight3A` and supports `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR` pipelines.
2. Teleop code starts the Limelight in `APRIL_TAG` mode and updates it every loop.
3. `PreseasonWebcamAuto` starts the Limelight in `COLOR` mode and uses Pinpoint-only localization.
4. `LimelightPinpointLocalizer` updates Pinpoint first, then attempts to fuse in a Limelight MT2 botpose reading.

## Repo-specific constraints

- If the camera is needed for color detection, prefer `Constants.pinpointFollower(...)` over `customLocalizerFollower(...)`.
- `Limelight.LLUpdate()` must be called regularly to keep results fresh and disconnected-state tracking meaningful.
- Vision results can be absent or invalid; code must tolerate null or invalid readings.
- Distance helpers in `Limelight.java` still use zeroed placeholder mounting constants and should not be trusted until measured values are filled in.

## Rules

- Keep pipeline choice explicit in the op mode.
- Avoid mixing color-detection logic with the custom Limelight fusion localizer unless the camera pipeline plan is revisited.
- When adjusting pose fusion, keep coordinate-system conversion explicit; this repo uses `FTCCoordinates.INSTANCE.convertToPedro(...)`.
- Do not introduce silent pose overwrites without checking for invalid or missing vision data.

## Example decisions

- Need AprilTag-assisted pose estimate while driving: consider the custom localizer path.
- Need color detection in autonomous: keep Pinpoint as the follower localizer and use the Limelight separately for vision.
- Need tag metadata or botpose telemetry: extend the `Limelight` wrapper rather than scattering direct SDK calls.
