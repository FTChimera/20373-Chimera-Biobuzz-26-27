---
name: localization-and-vision
description: Use this skill for Pinpoint localization, Limelight results, explicit relocalization, and vision-assisted paths.
---

# Localization and Vision

The default Pedro follower is built by
`RobotConstants.createFollower(HardwareMap)` (surfaced through
`Constants.createFollower(...)`) with Pinpoint localizer configuration copied
from the active profile's `PINPOINT_LOCALIZER_CONSTANTS`. Pinpoint remains the
normal, continuously-updated drive pose source; Limelight is a separate,
opt-in correction source that is never applied automatically.

## Current pose update path

- `MecanumDrive.periodic()` snapshots `lastPose = follower.getPose()` and then
  calls `follower.update()`. `RobotDrive.periodic(State)` calls
  `mecanumDrive.periodic()` first, then `updateFollowerMode()`, before
  handling manual teleop drive input.
- `Robot.periodic(State state)` iterates every registered mechanism in
  `Class`-name sorted order and calls `mechanism.periodic(state)` on each, so
  `RobotDrive` and `Vision` are both refreshed once per aggregate cycle; there
  is exactly one follower-update call path.
- `Robot.getPoseCompensatedForVelocity()` copies the current drive pose and
  adds the current `Vector` velocity's X/Y components directly (no explicit
  `dt` scaling; the commented-out `.times(helper.getDt() / 1000.0)` shows this
  was deliberately simplified). It backs both `Shooter` targeting and
  autonomous end-pose persistence.
- `Robot.saveEndVals()` only runs the autonomous branch when `getRobotMode()`
  is `AUTONOMOUS` **and** `lastState != null` (i.e. `periodic(state)` ran at
  least once); it then writes the compensated pose to
  `Blackboard.Caption.POSE` and the last state's alliance to
  `Blackboard.Caption.ALLIANCE`. In teleop mode it instead resets every
  `Blackboard.Item` except `DRIVER`. `tele/TeleOp.getStartingPoseAndSetAlliance()`
  reads Blackboard pose/alliance at `runOpMode()` start, before `waitForStart()`.

## Vision's role is read-only until explicitly forced

`Vision.periodic(State)` calls `updateRobotOrientation(heading)` and `update()`
only — it refreshes cached Limelight results and heading but the line that
would call `forceRelocalize(...)` is commented out in source. The only path
that writes a vision-derived pose onto the follower is
`Robot.forceRelocalize()` -> `Vision.forceRelocalize(RobotDrive)`, which
switches to `APRIL_TAG`, requires a currently valid result, blends the MT2 pose
with the existing drive pose using a fixed `0.3`/`0.7` weighting
(`Pose.linearCombination`), restores the previous pipeline, and returns whether
it actually applied a correction. Call it deliberately (e.g. once at a known
AprilTag-visible waypoint), not every loop.

`pedroPathing/custom/LimelightPinpointLocalizer.java` is a separate,
`@Deprecated`, unregistered `Localizer` implementation that wraps a
`PinpointLocalizer` and blends a Limelight MT2 pose directly inside its own
`update()` (wrapped in a try/catch that silently ignores failures). It is not
constructed by `RobotConstants.createFollower(...)`; changing it has no effect
on the active follower unless that builder is intentionally rewired to use it
instead of the plain Pinpoint localizer.

## BioBuzz path perception

`IntakeBallPath.generateIntakePathFromCam(follower, robotPose, vision, numBalls,
detectors)` is the current neural-detector path generator (see
`limelight-wrapper-and-pipelines` for its scoring/selection details). It
converts robot-relative forward/lateral offsets into field-frame poses through
`MathUtils.offsetPoseInRobotFrame(robotPose, forward, lateral,
Constants.FTC_COORDINATE_SYSTEM)`, so the coordinate system conversion is
always explicit and centralized rather than implicit inside a drive update.
Camera mount geometry, distance/lateral limits, merge distance, and the score
weights are `IntakeBallPath` constants that require on-robot calibration before
trusting the generated chain in a match.

Do not add a second periodic path that mutates drive pose from vision data;
`Robot.forceRelocalize()` is the single sanctioned entry point.

## Heading frame and Pinpoint calibration status

`MecanumDrive.headingOffset` (seeded from
`ROBOT_DRIVE_PARAMS.INITIAL_HEADING_OFFSET`) is subtracted from the Pinpoint
heading before field-centric teleop math (`shapeManualDriveVector(...)` and
the follower path both apply it), and `zeroHeadingOffset()`/`resetHeading(...)`
are the only ways to change it — vision correction never touches this offset,
only the underlying Pinpoint/follower pose. `Constants.createIMU(hardwareMap)`
initializes a separate `IMU` device (`RevHubOrientationOnRobot`) used only by
standalone hardware test op modes (`LLTester`, `MTeleWIntakeTest`,
`FieldCentricMecanumTeleOp`, `MecanumTeleOp`, `tests/ftcSdk/IMUTest`) — it is
not part of the `Robot`/`RobotDrive`/`MecanumDrive` follower path, which gets
heading exclusively from the Pinpoint-backed follower.

`RobotConfigurations.FirstRobot()` has real Pinpoint pod offsets
(`forwardPodY = -0.8704944520484759`, `strafePodX = -7.450610543799215`, both
encoder directions `REVERSED`) but zero drive velocity constants and default
`FollowerConstants()` — so Pinpoint distance/heading measurement itself is
wired to the physical robot, while follower tuning (translational/heading
PIDF, centripetal scaling) is not. `OldDECODERobotTesting()` carries the
previous season's fully-tuned follower constants as a reference profile; it is
marked for deletion once `FirstRobot()` is fully tuned. Any localization
accuracy work should first confirm which profile `Constants.currentBot`
selects before assuming a given set of Pinpoint/follower numbers is active.

## Summary of who touches the drive pose

| Caller | Reads pose | Writes pose |
| --- | --- | --- |
| `MecanumDrive.periodic()` | via `follower.getPose()` (cached to `lastPose`) | indirectly, via `follower.update()` |
| `Vision.periodic(State)` | no | no (relocalize call is commented out) |
| `Robot.forceRelocalize()` | via `requireDrive()` | yes, blended `0.3`/`0.7` |
| `Robot.saveEndVals()` | via `getPoseCompensatedForVelocity()` | no (writes `Blackboard`, not the follower) |

Keep this table in mind when tracing a reported localization bug: only two
call sites in current source ever mutate the live follower pose outside of
`follower.update()` itself — `forceRelocalize()` and direct `setPose(...)`
calls from op mode/test code.
