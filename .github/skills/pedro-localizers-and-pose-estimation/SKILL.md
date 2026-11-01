---
name: pedro-localizers-and-pose-estimation
description: Use this skill for Pinpoint setup, Pedro pose flow, and camera-assisted pose estimation.
---

# Pedro Localizers and Pose Estimation

`RobotConstants.createFollower(HardwareMap)` is the single place a Pedro
`Follower` is assembled. It builds three pieces from profile fields, then
hands them to `FollowerBuilder`:

- `FollowerConstants` from `DRIVE_CONSTANTS`, with
  `.headingPIDFCoefficients(HEADING_PIDF_COEFFICIENTS.getAsPedroPathing())`
  and `.centripetalScaling(CENTRIPETAL_SCALING)` applied on top.
- `MecanumConstants` from `MAX_POWER`, the four drive motor names/directions,
  and `X_VELOCITY`/`Y_VELOCITY`.
- `PinpointConstants` from `PINPOINT_LOCALIZER_CONSTANTS`, with
  `.distanceUnit(DistanceUnit.INCH)` and
  `.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)`
  applied on top of the profile's pod offsets/encoder directions.

`PathConstraints` comes from `T_VALUE_CONSTRAINT`, `TIMEOUT_CONSTRAINT`,
`BRAKING_STRENGTH`, and `BRAKING_START`. Keep the Pinpoint hardware name, pod
offsets (`forwardPodY`, `strafePodX`), encoder directions, and follower tuning
in `RobotConfigurations` (`FirstRobot()` / `OldDECODERobotTesting()`), not
scattered in op mode code.

## Pose update order

`MecanumDrive.periodic()` does exactly two things in order: snapshot
`lastPose = follower.getPose()`, then call `follower.update()`. `RobotDrive`
(a `Mechanism`) invokes `mecanumDrive.periodic()` as the first statement of its
own `periodic(State)`, before follower-mode bookkeeping or manual-drive input
handling. `Robot.periodic(state)` calls every mechanism's `periodic(state)`
once per aggregate cycle in class-name-sorted order — do not add a second,
competing `follower.update()` call anywhere else when using the aggregate
`Robot`/`RobotDrive` path.

## Vision interaction

`Vision.periodic(State)` calls `updateRobotOrientation(currentHeading)` and
`update()` to refresh cached Limelight/MT2 results; it does not write those
results back into the Pedro pose during periodic (the `forceRelocalize(...)`
call in `Vision.periodic` is commented out in source). `Robot.forceRelocalize()`
is the deliberate entry point: it requires a currently valid AprilTag result,
blends the MT2 pose with the existing follower pose using a fixed `0.3`
vision-weight / `0.7` existing-pose-weight low-pass filter
(`Pose.linearCombination(...)`), and writes the blended result with
`drive.setPose(...)`. Call it only when appropriate (e.g. a known tag-visible
moment), not every loop.

`pedroPathing/custom/LimelightPinpointLocalizer` is a separate, `@Deprecated`
`Localizer` implementation. It delegates almost every `Localizer` method
(`getPose`, `getVelocity`, `getVelocityVector`, `setStartPose`, `setPose`,
`getTotalHeading`, the multiplier getters, `resetIMU`, `getIMUHeading`,
`isNAN`) straight through to an internal `PinpointLocalizer`; only its
`update()` differs — it calls `pinpoint.update()`, then tries to read a fresh
Limelight MT2 botpose and overwrite the Pinpoint pose with it directly (no
blending), swallowing any exception silently. It is never constructed by
`RobotConstants.createFollower(...)`, so it is inert unless a follower is
rewired to use it in place of the plain `PinpointLocalizer`.

## Alliance-aware pose selection and handoff

`pedroPathing/AlliancePoses` stores a mirrored `redAlliancePose`/
`blueAlliancePose` pair and an internal `AllianceColor color` (defaults `RED`);
`pose()` returns `Red()` or `Blue()` (each a defensive `.copy()`) based on the
current color, and `setAlliance(...)` mutates that color. `Robot`'s
constructor reads `Blackboard.Caption.ALLIANCE` (defaulting to `RED` if unset)
and calls `Common.init(color)`, which is how planner poses such as
`Common.park` and route `AlliancePoses` fields become alliance-correct before
any mechanism periodic runs. Autonomous pose persistence itself is owned by
`Robot.saveEndVals()` (see `opmode-variants-and-pose-handoff`); teleop simply
consumes the resulting Blackboard pose/alliance at startup.

## Encoder resolution and distance units are forced, not profile-chosen

`RobotConstants.createFollower(...)` always applies
`.distanceUnit(DistanceUnit.INCH)` and
`.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)`
on top of whatever `PinpointConstants` a profile supplies — a profile can
still override hardware name, pod offsets, and encoder directions, but cannot
change distance unit or pod type through `RobotConfigurations` alone; that
would require editing `createFollower(...)` itself. Keep this in mind before
documenting a profile as using different Pinpoint pods.

## Standalone IMU vs. follower heading

A plain `IMU` device (`Constants.createIMU(hardwareMap)`) exists separately
from the Pinpoint-backed follower heading and is only wired into standalone
hardware test op modes (`tests/LLTester`, `tests/MTeleWIntakeTest`,
`tests/FieldCentricMecanumTeleOp`, `tests/MecanumTeleOp`,
`tests/ftcSdk/IMUTest`) — none of `Robot`, `RobotDrive`, or `MecanumDrive`
construct or read it. Any change to `IMU_LOGO_FACING_DIRECTION`/
`IMU_USB_FACING_DIRECTION` in `RobotConstants` therefore affects only those
standalone tests, not the Pedro follower's pose estimate.

## Quick reference: who owns each localization concern

- Pinpoint hardware wiring (name, pod offsets, encoder directions): profile
  builders in `RobotConfigurations`.
- Encoder resolution / distance unit (forced to `goBILDA_4_BAR_POD` / inches):
  `RobotConstants.createFollower(...)`.
- Per-loop pose update: `MecanumDrive.periodic()` -> `follower.update()`,
  driven by `RobotDrive.periodic(State)` inside `Robot.periodic(state)`.
- One-shot vision correction: `Vision.forceRelocalize(RobotDrive)` via
  `Robot.forceRelocalize()`.
- Cross-op-mode pose persistence: `Robot.saveEndVals()` writing
  `Blackboard.Caption.POSE`/`ALLIANCE`.
- Deprecated, unused-by-default alternative localizer:
  `pedroPathing/custom/LimelightPinpointLocalizer`.

Treat any change to localization behavior as touching at least two of these
layers (e.g. a new blend ratio touches `Vision.forceRelocalize`, while a new
pod offset touches `RobotConfigurations`); update both the code and this
guide together.
