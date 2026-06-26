---
name: localization-and-vision
description: Use this skill when asked to work on Limelight, AprilTag pose updates, Pinpoint localization, color pipelines, or vision-assisted autonomous behavior in this repo.
---

# Localization and Vision for This Robot Code

This repository currently mixes GoBILDA Pinpoint odometry with Limelight usage, depending on the mode.

## Relevant files

- `Constants.java`
- `library/hardware/webcam/Limelight.java`
- `library/hardware/webcam/LogitechWebcam.java`
- `pedroPathing/custom/LimelightPinpointLocalizer.java`
- `tests/PreseasonWebcamAuto.java`
- `opModes/IvyTeleOp.java`
- `opModes/TeleOp.java`

## Current usage patterns

1. `Limelight` wraps `Limelight3A` and supports `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR` pipelines.
2. Teleop code creates the wrapper through `Constants.createLimelight(hardwareMap)`, starts it in `APRIL_TAG` mode, and updates it every loop.
3. `tests/PreseasonWebcamAuto` starts the raw Limelight device in `COLOR` mode as a proof-of-concept vision-following test.
4. Even when a test bypasses the wrapper for behavior, the repo now prefers sourcing hardware names from the wrapper params or robot profile instead of duplicating literals.
5. `LogitechWebcam` is a separate AprilTag / VisionPortal wrapper that can also be profile-managed through `Constants.createWebcam(...)`.
6. `LimelightPinpointLocalizer` exists as a deprecated custom fusion helper; it is not the default follower path.

## Vision-wrapper configuration pattern

Vision wrappers should follow the same pattern as other mechanisms:

- define wrapper-specific helper params on the wrapper when they belong there, such as `LIMELIGHT_PARAMS`
- extend `Mechanism` when the wrapper should participate in the standard robot-subsystem flow
- keep the constructor shape `(HardwareMap, RobotConstants)`
- register the wrapper in the active robot profile with `.addMechanism(...)`
- expose construction through `Constants.createLimelight(...)`
- avoid direct wrapper construction in every op mode

That keeps vision wrappers aligned with the same profile-aware construction path as the rest of the robot.

## Repo-specific constraints

- If the camera is needed for color detection, prefer the normal Pinpoint follower path over custom fusion.
- `Limelight.LLUpdate()` must be called regularly to keep results fresh and disconnected-state tracking meaningful.
- Vision results can be absent or invalid; code must tolerate null or invalid readings.
- Distance helpers in `Limelight.java` still use placeholder mounting constants and should not be trusted until measured values are filled in.
- If a vision wrapper is optional, it should decide whether it is enabled before calling `hardwareMap.get(...)`.

## Rules

- Keep pipeline choice explicit in the op mode.
- Avoid mixing color-detection logic with the custom Limelight fusion localizer unless the camera pipeline plan is intentionally revisited.
- When adjusting pose fusion, keep coordinate-system conversion explicit.
- Do not introduce silent pose overwrites without checking for invalid or missing vision data.
- If a new camera wrapper is added, make it fit the `Mechanism` + `RobotConstants.createMechanism(...)` pattern and expose a `Constants.create...(...)` helper if higher-level code should call it directly.

## Example decisions

- Need AprilTag-assisted pose estimate while driving: consider whether the deprecated custom localizer path is really necessary, because the default follower still uses Pinpoint only.
- Need color detection in autonomous: keep Pinpoint as the follower localizer and use the Limelight separately for vision.
- Need tag metadata or botpose telemetry: extend the `Limelight` or `LogitechWebcam` wrapper rather than scattering direct SDK calls.
- Need a new vision mechanism or camera wrapper: add it to the normal profile-managed construction flow before wiring it into op modes.
