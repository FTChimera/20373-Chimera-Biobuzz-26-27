---
name: limelight-wrapper-and-pipelines
description: Use this skill when asked to work directly on the TeamCode Limelight wrapper, switch pipelines, or consume color, AprilTag, and detector results through the existing abstraction.
---

# Limelight Wrapper and Pipelines

Use this skill for Limelight-specific work that is narrower than the broader localization-and-vision skill.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/PreseasonWebcamAuto.java`

## Existing abstraction

- `Constants.createLimelight(hardwareMap)` is the normal factory entry point.
- `Constants` owns the shared `Limelight.LIMELIGHT_PARAMS` instance so Dashboard/panels can edit it centrally.
- `Limelight.Pipeline` currently exposes `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR`.
- `start(...)` chooses a pipeline and starts the FTC `Limelight3A` device.
- `LLUpdate()` pulls the latest result, tracks disconnect state, and caches fiducials, colors, and detectors.
- `updateCommand()` exposes a Pedro Ivy command for regular updates.

## Repo conventions

- Teleop starts the camera in `APRIL_TAG` mode.
- `PreseasonWebcamAuto` starts the camera in `COLOR` mode.
- Code usually reads from the wrapper’s cached fields instead of calling the raw SDK directly from multiple places.

## Params and creation pattern

When working on `Limelight` or adding a similar wrapper:

1. keep the params type with the wrapper class
2. keep the shared editable params instance in `Constants`
3. expose construction through `Constants.create...(...)`
4. avoid direct wrapper construction in op modes unless the task is specifically refactoring the factory surface

The mechanism/wrapper instance can store `params` normally, but the shared config instance should stay centralized in `Constants`.

## Rules

- Call `LLUpdate()` regularly if the Limelight is active.
- Keep pipeline selection explicit in the op mode or mechanism that owns the behavior.
- Extend the wrapper when new camera behavior is needed rather than scattering `Limelight3A` calls across op modes.
- Tolerate missing or invalid results; `LLUpdate()` already handles absent data as a normal runtime condition.
- If new vision tuning fields are added, add them to `LIMELIGHT_PARAMS` and keep the shared instance in `Constants`.

## Current limitations to remember

- The distance helper uses placeholder mounting constants, so those values are not trustworthy until measured.
- `isDisconnected` depends on repeated updates and the internal timer.
- The wrapper supports operating in a no-device mode via the no-arg constructor; guard logic accordingly.

## Good extension points

- Add helper methods that interpret fiducial, color, or detector results.
- Add telemetry-friendly summary methods.
- Add pipeline-specific convenience behavior while keeping pipeline numbers hidden behind the enum.
- If adding a new camera wrapper class, mirror the same nested-params plus `Constants.create...(...)` pattern used here.

