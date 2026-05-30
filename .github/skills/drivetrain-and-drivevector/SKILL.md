---
name: drivetrain-and-drivevector
description: Use this skill when asked to change teleop drive math, field-centric behavior, stick curves, or the interaction between `DriveVector`, `Drivetrain`, and `MecanumDrive`.
---

# Drivetrain and DriveVector

This skill covers the team’s custom teleop-drive stack.

## Key files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/MecanumTeleOp.java`

## Current architecture

1. `DriveVector` stores forward, strafe, rotate, and robot heading values.
2. `DriveVector.applyStickCurve(...)` applies the selected joystick curve and flips forward for driver-friendly stick behavior.
3. `Drivetrain.setTeleOpDrive(...)` optionally rotates the vector for field-centric drive and writes power to the four `Motor` wrappers.
4. `MecanumDrive` obtains its follower and drivetrain through `Constants`-based creation paths.
5. `MecanumDrive.periodic()` updates the Pedro follower and caches `lastPose`.
6. Teleop code usually calls `MecanumDrive.setTeleOpDrive(new DriveVector(...))` while Pedro handles localization.

## Repo-specific conventions

- The project defaults to `Constants.DRIVE_FIELD_RELATIVE` for field-centric selection.
- `TeleOp` and `IvyTeleOp` square the stick inputs with `x -> Math.signum(x) * x * x`.
- Heading reset is usually mapped to `gamepad1.back`.
- `MecanumDrive` is the public drive-facing mechanism; the raw `Follower` is not the normal teleop entry point in team code.
- Drive motor directions in robot profiles now use `library.hardware.actuators.Direction`; `Drivetrain` applies them through `Motor.setDirection(...)`.

## How drive fits the new creation pattern

- Non-drive mechanisms expose their shared params instances centrally from `Constants`.
- Drive profile and follower configuration live in `RobotConstants` / `RobotConfigurations`.
- `Constants.createDrivetrain(hardwareMap)` is the central creation point for the low-level drive mechanism.
- Higher-level drive code such as `MecanumDrive.pinpointLocalizer(...)` should keep delegating to those central factories instead of rebuilding hardware inline.

If a new drive-adjacent mechanism is added, give it its own params type and central `Constants` factory unless the data is truly robot-profile data that belongs in `RobotConstants`.

## Rules

- Keep drive-input shaping in one place: `DriveVector` or the selected stick curve, not scattered sign flips across op modes.
- If field-centric math changes, verify both `DriveVector.rotateFieldCentric()` and the heading passed in from `MecanumDrive`.
- Preserve `periodic()` calling `follower.update()` exactly once per loop path.
- Prefer extending `MecanumDrive` APIs over having op modes reach inside `drivetrain` and `follower` directly.
- Keep drivetrain profile data in `RobotConstants`; keep non-drive shared params in `Constants`.

## Common edit targets

- Change response curve or deadband behavior: `DriveVector.java`
- Change motor-power mapping or field-centric handling: `Drivetrain.java`
- Change public teleop-facing drive behavior or pose resets: `MecanumDrive.java`
- Change drive hardware names, directions, or follower tuning: `RobotConfigurations.java` / `RobotConstants.java`

## Validation ideas

- Check that forward stick still drives the robot in the intended direction.
- Check field-centric orientation after a heading reset.
- Use `tests/MecanumTeleOp.java` when you need a direct SDK-based robot-centric / field-centric check or want to flip individual motor directions interactively.
- Use `MotorDirectionTest` when you want to validate the constants-driven drivetrain profile and intake direction at low speed.

