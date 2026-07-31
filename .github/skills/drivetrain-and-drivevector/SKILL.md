---
name: drivetrain-and-drivevector
description: Use this skill when asked to change teleop drive math, field-centric behavior, stick curves, or the interaction between `DriveVector`, `Drivetrain`, and `MecanumDrive`.
---

# Drivetrain and DriveVector

This skill covers the team’s custom teleop-drive stack.

## Key files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/OpModeHelper.kt`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/Driver.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/FieldCentricMecanumTeleOp.java`

## Current architecture

1. `DriveVector` stores forward, strafe, rotate, and robot heading values.
2. `DriveVector.applyStickCurve(...)` mutates the vector in place and applies the selected joystick curve without flipping any axes.
3. `DriveVector.rotateFieldCentric()` currently rotates by `-robotHeading`, which is the current repo convention after the field-centric fix.
4. `Drivetrain.setTeleOpDrive(...)` copies the incoming vector, applies the configured stick curve, explicitly negates forward, optionally rotates for field-centric drive, and writes power to the four `Motor` wrappers.
5. `MecanumDrive` creates its follower from the active `RobotConstants` profile and pairs it with a `Drivetrain` built from the same `bot` instance.
6. `MecanumDrive.periodic()` currently snapshots `lastPose = follower.getPose()` and then calls `follower.update()`, so `lastPose` is the pre-update pose for that loop.
7. `MecanumDrive.setTeleOpDrive(new DriveVector(...))` routes through `Drivetrain` or Pedro's follower-drive path according to the selected profile's `USE_FOLLOWER_DRIVE` setting.

## Repo-specific conventions

- Driver preference for field-centric versus robot-centric mode is stored in `configs.Blackboard` through `Blackboard.Caption.DRIVER` and the `Driver` enum.
- `TeleOp` and `IvyTeleOp` now set their stick curve once on `MecanumDrive` via `drive.setStickCurve(x -> Math.signum(x) * x * x)` and then pass raw stick values into `new DriveVector(...)`; the forward-axis inversion now happens inside `Drivetrain` or at the raw-`Follower` call site instead of inside `DriveVector`.
- Heading reset is mapped to `gamepad1.back` in the shared teleop bases; `tests/FieldCentricMecanumTeleOp.java` uses `gamepad1.options`.
- IMU lookup and REV Hub orientation are profile-owned through `RobotConstants`; use `Constants.createIMU(hardwareMap)` to initialize the active-profile IMU, then reset yaw explicitly in the op mode.
- `MecanumDrive` is the public drive-facing mechanism; the raw `Follower` is not the normal teleop entry point in team code.
- Drive motor directions in robot profiles now use `library.hardware.actuators.Direction`; `Drivetrain` applies them through `Motor.setDirection(...)`.
- FirstRobot's drivetrain hardware wiring and Pinpoint setup are ready. Its
  Pedro drive-tuning fields remain
  unconfigured (`X_VELOCITY` / `Y_VELOCITY` are zero and the follower
  coefficients are defaults), so do not treat follower-drive behavior as
  calibrated.
- `tests/MecanumTeleOp.java` now pulls drivetrain motor names and initial directions from `Constants.currentBot` rather than keeping separate stale test-only drive strings.
- `tests/FieldCentricMecanumTeleOp.java` is still a direct SDK example that hardcodes right-side motor reversal, so treat it as a manual reference rather than the profile-owned drive stack.
- `MotorDirectionTest` is now a live low-speed loop that keeps drivetrain/intake direction telemetry updating while you flip directions, so drive-sign edits should stay compatible with that constants-driven sanity check.
- `OpModeHelper.kt` is available if a drive-focused op mode needs manual Lynx bulk-cache clearing and loop-time tracking without inventing another helper.

## How drive fits the new creation pattern

- Drive profile and follower configuration live in `RobotConstants` / `RobotConfigurations`.
- `Drivetrain` and `MecanumDrive` both participate in the shared `Mechanism` creation flow.
- `Constants.createTeleOpFollower(hardwareMap)` is the top-level convenience wrapper around `currentBot.createMechanism(hardwareMap, MecanumDrive.class)`, and `Constants.createDrivetrain(hardwareMap)` is the lower-level wrapper around `Drivetrain.class`.
- `RobotConstants.configureDriveMechanisms()` is the profile helper that enables the normal drive stack (`MecanumDrive` plus `Drivetrain`).
- Higher-level drive code should keep delegating to those normal factories instead of rebuilding hardware inline.

If a new drive-adjacent mechanism is added, make it follow the `Mechanism` + `(HardwareMap, RobotConstants)` constructor pattern and register it in the robot profile unless the change is purely a reusable math helper.

## Rules

- Keep drive-input shaping in one place: `DriveVector` or the selected stick curve, not scattered sign flips across op modes.
- If field-centric math changes, verify both `DriveVector.rotateFieldCentric()` and the heading passed in from `MecanumDrive`.
- Preserve `periodic()` calling `follower.update()` exactly once per loop path.
- Prefer extending `MecanumDrive` APIs over having op modes reach inside `drivetrain` and `follower` directly.
- Keep drivetrain profile data in `RobotConstants`; keep subsystem construction on the existing `Constants` / `RobotConstants.createMechanism(...)` path.
- If a change touches driver defaults, update the `Driver` enum and any `Blackboard`-based preference flow rather than inventing a second field-centric toggle source.

## Common edit targets

- Change response curve or deadband behavior: `DriveVector.java`
- Change motor-power mapping or field-centric handling: `Drivetrain.java`
- Change public teleop-facing drive behavior or pose resets: `MecanumDrive.java`
- Change drive hardware names, directions, or follower tuning: `RobotConfigurations.java` / `RobotConstants.java`
- Change IMU hardware name or hub orientation: `RobotConfigurations.java` / `RobotConstants.java`
- Change centralized drivetrain validation wiring in the raw test op mode: `tests/MecanumTeleOp.java`

## Validation ideas

- Check that forward stick still drives the robot in the intended direction.
- Check field-centric orientation after a heading reset.
- Verify the selected profile's `USE_FOLLOWER_DRIVE` value and test the resulting drive path.
- Check both follower-drive and manual-drive behavior if `USE_FOLLOWER_DRIVE` changes.
- Use `tests/MecanumTeleOp.java` when you need a direct SDK-based robot-centric / field-centric check or want to flip individual motor directions interactively while staying aligned with the active drivetrain profile names.
- Use `MotorDirectionTest` when you want to validate the constants-driven drivetrain profile and intake direction at low speed.
