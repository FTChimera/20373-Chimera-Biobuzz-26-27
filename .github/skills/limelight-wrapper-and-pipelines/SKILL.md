---
name: limelight-wrapper-and-pipelines
description: Use this skill when asked to work directly on the TeamCode Limelight wrapper, switch pipelines, or consume color, AprilTag, and detector results through the existing abstraction.
---

# Limelight Wrapper and Pipelines

Use this skill for Limelight-specific work that is narrower than the broader localization-and-vision skill.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Mechanisms.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/PreseasonWebcamAuto.java`

## Existing abstraction

- `Mechanisms.limelight(hardwareMap)` is the normal factory entry point.
- `Limelight.Pipeline` currently exposes `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR`.
- `start(...)` chooses a pipeline and starts the FTC `Limelight3A` device.
- `LLUpdate()` pulls the latest result, tracks disconnect state, and caches fiducials/colors/detectors.
- `updateCommand()` exposes a Pedro Ivy command for regular updates.

## Repo conventions

- Teleop starts the camera in `APRIL_TAG` mode.
- `PreseasonWebcamAuto` starts the camera in `COLOR` mode.
- Code usually reads from the wrapper’s cached fields instead of calling the raw SDK directly from multiple places.

## Rules

- Call `LLUpdate()` regularly if the Limelight is active.
- Keep pipeline selection explicit in the op mode or mechanism that owns the behavior.
- Extend the wrapper when new camera behavior is needed rather than scattering `Limelight3A` calls across op modes.
- Tolerate missing or invalid results; `LLUpdate()` already handles absent data as a normal runtime condition.

## Current limitations to remember

- The distance helper uses zeroed mounting constants, so those values are placeholders until measured.
- `isDisconnected` depends on repeated updates and the internal timer.
- The wrapper supports operating in a no-device mode via the no-arg constructor; guard logic accordingly.

## Good extension points

- Add helper methods that interpret fiducial, color, or detector results.
- Add telemetry-friendly summary methods.
- Add pipeline-specific convenience behavior while keeping pipeline numbers hidden behind the enum.

