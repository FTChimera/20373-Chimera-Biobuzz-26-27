---
name: drivetrain-and-drivevector
description: Use this skill for teleop drive math, follower modes, and DriveVector behavior.
---

# Drivetrain and DriveVector

Use this guide for driver translation, turning, field-centric behavior, and follower handoff.

## Current ownership

- `RobotDrive` is the profile-managed drive mechanism.
- It wraps the configured `MecanumDrive` and follower behavior.
- `Robot` exposes drive helpers that delegate to `RobotDrive`.
- `tele/TeleOp` configures the robot, then calls `startTeleOpDrive()`.
- Autonomous planner commands use the same aggregate robot and configured drive.

## Mechanism lifecycle

- `RobotDrive` implements `Mechanism`.
- Its periodic signature is `periodic(State)`.
- Its stop signature is `stop()`.
- `Robot.periodic(State)` is the only aggregate dispatch path.
- The state mode must equal the robot's configured mode.
- Do not add a second mode parameter to periodic methods.

## Driver control flow

1. Create `DualGamepad` from FTC `gamepad1` and `gamepad2`.
2. Put its `GpBindings` view on the teleop `State`.
3. Call `DualGamepad.update()` once per loop.
4. Call `Robot.periodic(state)` after inputs are current.
5. Read or set drive intent through the drive mechanism APIs.

## Drive vectors

- `DriveVector` represents translational and rotational driver intent.
- Keep coordinate conventions explicit at every conversion boundary.
- Use robot helpers to set teleop drive instead of bypassing profile configuration.
- `Robot.setTeleOpDrive(vector)` uses normal configured behavior.
- `Robot.setTeleOpDrive(vector, fieldCentric)` makes the choice explicit.
- Do not construct a field-relative vector from raw stick axes without checking signs and axes.

## Stick curves

- `Robot.setDriveStickCurve(...)` delegates to `RobotDrive`.
- `TeleOp` currently installs a signed nonlinear curve before match start.
- Configure the curve once during initialization, not repeatedly in the loop.
- Keep curves monotonic and preserve zero at zero.
- Test deadzone and curve behavior with the existing gamepad tests where possible.

## Follower modes

- `RobotDrive` controls transitions between teleop and follower operation.
- Start driver operation with `Robot.startTeleOpDrive()`.
- Set the starting pose before driving when localization requires one.
- Use `Robot.setStartingPose(...)` or `Robot.setPose(...)`, not private follower internals.
- Preserve a single owner for drivetrain commands while a path is active.
- Coordinate scheduled follower commands with manual input so they do not fight.

## Heading and localization

- `Robot.zeroDriveHeading()` delegates to the drive heading-offset operation.
- `Robot.getPose()` returns the configured drive pose.
- `Robot.getPoseCompensatedForVelocity()` is used for autonomous pose handoff.
- Vision does not implicitly rewrite pose during normal periodic work.
- Apply a vision correction through the explicit relocalization path when appropriate.

## Profile requirements

- A drive call that uses `requireDrive()` requires a registered `RobotDrive`.
- Check the active `RobotConstants` configuration before adding a drive dependency.
- Hardware names, directions, and localizer offsets belong in the profile.
- Do not hide profile errors by creating an unconfigured drivetrain locally.

## Validation

- Compile TeamCode after changing Java drive code.
- Run focused existing unit tests for changed pure math.
- Test direction, rotation sign, and field-centric orientation on the robot.
- Confirm teleop starts with a Blackboard pose when autonomous has saved one.
- Confirm stopping zeroes or otherwise safely stops drive output.
- Verify no code path updates drive from stale gamepad input.

## Troubleshooting

- First verify active profile motor names and directions.
- Then verify localization pose and heading before changing stick signs.
- Confirm the drive is in teleop mode before diagnosing manual controls.
- Confirm follower commands are not still scheduled during driver control.
- Inspect the configured stick curve before adding another response transform.
- Use telemetry for pose, velocity, and requested vector during physical tests.
- Change one sign or convention at a time and re-test.
- Keep profile calibration separate from user-control policy.
- Recheck autonomous handoff after changing heading or coordinate logic.
- Document any intentional driver preference in the teleop entry.

## Change boundaries

- Keep motor hardware access inside the configured drive implementation.
- Keep driver policy in the teleop layer or explicit drive API.
- Keep path geometry in planner modules rather than joystick code.
- Revalidate all three boundaries after a drive-interface change.
