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
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`

## Existing abstraction

- `Constants.createLimelight(hardwareMap)` is the normal factory entry point.
- `Limelight` extends `Mechanism`, so the normal factory path is `Constants.createLimelight(...)` → `currentBot.createMechanism(...)`.
- `Limelight.Pipeline` currently exposes `APRIL_TAG`, `COLOR`, and `NEURAL_DETECTOR`.
- `start(...)` chooses a pipeline and starts the FTC `Limelight3A` device.
- `LLUpdate()` pulls the latest result, tracks disconnect state, and caches fiducials, colors, and detectors.
- `updateCommand()` exposes a Pedro Ivy command for regular updates.

## Repo conventions

- Teleop starts the camera in `APRIL_TAG` mode.
- `tests/PreseasonWebcamAuto` starts the camera in `COLOR` mode as a proof-of-concept validation flow.
- Code usually reads from the wrapper’s cached fields instead of calling the raw SDK directly from multiple places.

## Params and creation pattern

When working on `Limelight` or adding a similar wrapper:

1. keep wrapper-specific helper params with the wrapper class when they belong there
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
- If new vision tuning fields are added, keep them on `LIMELIGHT_PARAMS` or the wrapper itself rather than scattering them through op modes.
- If the wrapper can be disabled by profile selection, make sure it decides that before touching hardware.

## Current limitations to remember

- The distance helper uses placeholder mounting constants, so those values are not trustworthy until measured.
- `isDisconnected` depends on repeated updates and the internal timer.
- The wrapper supports operating in a no-device mode by setting `params.ISBEINGUSED` false before hardware lookup; guard logic accordingly.

## Good extension points

- Add helper methods that interpret fiducial, color, or detector results.
- Add telemetry-friendly summary methods.
- Add pipeline-specific convenience behavior while keeping pipeline numbers hidden behind the enum.
- If adding a new camera wrapper class, mirror the same `Mechanism` + `(HardwareMap, RobotConstants)` + `Constants.create...(...)` pattern used here.

