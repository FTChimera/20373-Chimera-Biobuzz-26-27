---
name: pedro-localizers-and-pose-estimation
description: Use this skill when asked to work on Pinpoint setup, custom localizers, pose estimation flow, or Pedro localization updates in this repository.
---

# Pedro Localizers and Pose Estimation

Use this skill for localization-specific Pedro work that is narrower than general path building.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/PreseasonWebcamAuto.java`

## Current localization flow

1. The default follower path uses Pinpoint through `RobotConstants.createFollower(...)`.
2. `MecanumDrive.pinpointLocalizer(...)` is the normal drive entry point.
3. `LimelightPinpointLocalizer` exists as a custom fusion localizer but is marked `@Deprecated`.
4. `MecanumDrive.periodic()` keeps the follower pose fresh and mirrors it into `lastPose`.
5. `Blackboard.pose` may persist the latest pose across op modes.

## Rules

- Keep Pinpoint hardware naming, offsets, and encoder directions in `RobotConstants` / `RobotConfigurations`.
- When changing localization math, be explicit about coordinate-system conversion.
- If the Limelight pipeline is needed for vision tasks, prefer Pinpoint-only following unless the fusion plan is intentionally revisited.
- Do not add multiple competing pose-update loops for the same follower.

## Things to watch

- `LimelightPinpointLocalizer.update()` first updates Pinpoint, then tries to override pose from Limelight MT2 botpose.
- The fusion class currently swallows exceptions, so missing vision data is treated as a normal condition.
- Some code comments explicitly warn against using custom fusion when the camera is needed for color detection.

## Good edit targets

- Pinpoint geometry or encoder configuration: `RobotConfigurations.java` / `RobotConstants.java`
- Custom fusion logic: `LimelightPinpointLocalizer.java`
- Pose consumption in teleop/autonomous: `MecanumDrive.java` or the relevant op mode

