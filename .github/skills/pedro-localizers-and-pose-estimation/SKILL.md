---
name: pedro-localizers-and-pose-estimation
description: Use this skill when asked to work on Pinpoint setup, custom localizers, pose estimation flow, or Pedro localization updates in this repository.
---

# Pedro Localizers and Pose Estimation

Use this skill for localization work that is narrower than general autonomous/path construction.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/Blackboard.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Red_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Blue_TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/PreseasonWebcamAuto.java`

## Current localization flow

1. The default follower path is `Constants.createFollower(...)` → `Constants.currentBot.createFollower(...)`. The `OldDECODERobotTesting` and `FirstRobot` profiles exist in `RobotConfigurations`; `Constants.currentBot` selects the profile that provides follower configuration.
2. `RobotConstants.createFollower(...)` wires localization through `FollowerBuilder(...).pinpointLocalizer(PINPOINT_LOCALIZER_CONSTANTS)`.
3. Both current robot profiles call `.pinpointConstants(new PinpointConstants())`; the code does not currently set custom Pinpoint offsets or encoder-direction fields on that object.
4. `MecanumDrive` snapshots `follower.getPose()` into static `lastPose` and then calls `follower.update()` inside `periodic()`.
5. Shared autonomous-to-teleop pose handoff currently happens when `opModes/IvyAuton.java` writes `follower.getPose()` to `Blackboard.Caption.POSE` in `stop()`, and `variants/Red_TeleOp.java` / `Blue_TeleOp.java` read it back if it is not `(0,0,0)`.
6. `LimelightPinpointLocalizer` exists as a custom fusion localizer, is marked `@Deprecated`, and is not used by the default follower factory.
7. `tests/PreseasonWebcamAuto.java` keeps the normal follower and uses raw Limelight color results separately; it does not replace the default localizer.

## Configuration split to preserve

- Keep drivetrain names, motor directions, follower tuning, and Pinpoint constants in `RobotConstants` / `RobotConfigurations`.
- Keep non-drive subsystem behavior out of the robot-profile layer unless it truly varies by robot profile.
- If localization work also touches a vision wrapper, keep that wrapper on the `Mechanism` + `RobotConstants.createMechanism(...)` path.

The `OldDECODERobotTesting` and `FirstRobot` profiles exist. Localization changes that use `Constants.createLimelight(...)` or `Constants.createWebcam(...)` must ensure the profile selected by `Constants.currentBot` registers the required wrapper.

## Rules

- Keep Pinpoint hardware naming and localizer configuration in `RobotConstants` / `RobotConfigurations`.
- When changing fusion math, keep coordinate-system conversion explicit.
- Do not add multiple competing pose-update loops for the same follower.
- If a task needs Limelight color detection at the same time as following, follow the current repo pattern: keep the follower on Pinpoint and consume Limelight results separately unless the fusion plan is intentionally revisited.
- If you change auto-to-teleop pose handoff behavior, update both the writer (`IvyAuton.stop()`) and the readers (`Red_TeleOp` / `Blue_TeleOp`).

## Things to watch

- `LimelightPinpointLocalizer.update()` first updates Pinpoint, then pushes Pinpoint heading into `limelight.updateRobotOrientation(...)`, then tries to replace pose from `getBotpose_MT2()`.
- The fusion localizer converts Limelight data through FTC coordinates before storing the result back into Pedro coordinates.
- The fusion class swallows exceptions, so missing or invalid Limelight data is treated as a normal condition.
- Changes to `LimelightPinpointLocalizer` do not affect `Constants.createFollower(...)` unless you also rewire `RobotConstants.createFollower(...)`.

## Good edit targets

- Pinpoint geometry or profile selection: `RobotConfigurations.java` / `RobotConstants.java`
- Default follower wiring: `RobotConstants.createFollower(...)`
- Custom fusion logic: `LimelightPinpointLocalizer.java`
- Shared pose handoff behavior: `IvyAuton.java`, `Blackboard.java`, `Red_TeleOp.java`, `Blue_TeleOp.java`
- Raw Limelight-follow integration tests: `tests/PreseasonWebcamAuto.java`
