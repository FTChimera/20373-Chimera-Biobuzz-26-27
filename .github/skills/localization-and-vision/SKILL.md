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
2. Teleop code creates the wrapper through `Constants.createLimelight(hardwareMap)`, starts it in `APRIL_TAG` mode, and updates it every loop.
3. `PreseasonWebcamAuto` starts the Limelight in `COLOR` mode and uses Pinpoint-only localization.
4. `LimelightPinpointLocalizer` updates Pinpoint first, then attempts to fuse in a Limelight MT2 botpose reading.

## Vision-wrapper configuration pattern

Vision wrappers should follow the same pattern as other mechanisms:

- define a nested params type on the wrapper, such as `LIMELIGHT_PARAMS`
- keep the shared editable params instance in `Constants`
- expose construction through `Constants.createLimelight(...)`
- avoid direct wrapper construction in every op mode

That centralization is important because the team wants Dashboard/panels to edit vision params from one place.

## Repo-specific constraints

- If the camera is needed for color detection, prefer the normal Pinpoint follower path over custom fusion.
- `Limelight.LLUpdate()` must be called regularly to keep results fresh and disconnected-state tracking meaningful.
- Vision results can be absent or invalid; code must tolerate null or invalid readings.
- Distance helpers in `Limelight.java` still use placeholder mounting constants and should not be trusted until measured values are filled in.

## Rules

- Keep pipeline choice explicit in the op mode.
- Avoid mixing color-detection logic with the custom Limelight fusion localizer unless the camera pipeline plan is intentionally revisited.
- When adjusting pose fusion, keep coordinate-system conversion explicit.
- Do not introduce silent pose overwrites without checking for invalid or missing vision data.
- If a new camera wrapper is added, give it its own params type plus a `Constants.create...(...)` factory.

## Example decisions

- Need AprilTag-assisted pose estimate while driving: consider the custom localizer path.
- Need color detection in autonomous: keep Pinpoint as the follower localizer and use the Limelight separately for vision.
- Need tag metadata or botpose telemetry: extend the `Limelight` wrapper rather than scattering direct SDK calls.
- Need a new vision mechanism or camera wrapper: add its params type in the wrapper, its shared instance in `Constants`, and its factory in `Constants` before wiring it into op modes.
