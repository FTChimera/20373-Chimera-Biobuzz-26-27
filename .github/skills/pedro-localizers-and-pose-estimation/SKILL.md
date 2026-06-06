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

## Configuration split to preserve

- Keep Pinpoint geometry, hardware naming, encoder directions, and follower tuning in `RobotConstants` / `RobotConfigurations`.
- Keep non-drive subsystem behavior out of the robot-profile layer unless it truly varies by robot profile.
- If a localization task also touches a vision wrapper like `Limelight`, continue creating that wrapper through `Constants.createLimelight(...)` and the underlying `RobotConstants.createMechanism(...)` flow.

This repo intentionally separates Pedro/drive profile data from subsystem implementation details so the drivetrain profile does not become a dumping ground for unrelated robot behavior.

## Rules

- Keep Pinpoint hardware naming, offsets, and encoder directions in `RobotConstants` / `RobotConfigurations`.
- When changing localization math, be explicit about coordinate-system conversion.
- If the Limelight pipeline is needed for vision tasks, prefer Pinpoint-only following unless the fusion plan is intentionally revisited.
- Do not add multiple competing pose-update loops for the same follower.
- Do not move unrelated subsystem details into the robot-profile layer just because a localization task touches them.

## Things to watch

- `LimelightPinpointLocalizer.update()` first updates Pinpoint, then tries to override pose from Limelight MT2 botpose.
- The fusion class currently swallows exceptions, so missing vision data is treated as a normal condition.
- Some code comments explicitly warn against using custom fusion when the camera is needed for color detection.

## Good edit targets

- Pinpoint geometry or encoder configuration: `RobotConfigurations.java` / `RobotConstants.java`
- Custom fusion logic: `LimelightPinpointLocalizer.java`
- Pose consumption in teleop/autonomous: `MecanumDrive.java` or the relevant op mode
- Limelight wrapper params or factory behavior: `Constants.java` / `Limelight.java`

