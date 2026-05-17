---
name: drivetrain-and-drivevector
description: Use this skill when asked to change teleop drive math, field-centric behavior, stick curves, or the interaction between `DriveVector`, `Drivetrain`, and `MecanumDrive`.
---

# Drivetrain and DriveVector

This skill covers the team’s custom teleop-drive stack.

## Key files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/DriveVector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/TeleOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyTeleOp.java`

## Current architecture

1. `DriveVector` stores forward, strafe, rotate, and robot heading values.
2. `DriveVector.applyStickCurve(...)` applies the selected joystick curve and flips forward for driver-friendly stick behavior.
3. `Drivetrain.setTeleOpDrive(...)` optionally rotates the vector for field-centric drive and writes power to the four `Motor` wrappers.
4. `MecanumDrive.periodic()` updates the Pedro follower and caches `lastPose`.
5. Teleop code usually calls `MecanumDrive.setTeleOpDrive(new DriveVector(...))` while Pedro handles localization.

## Repo-specific conventions

- The project defaults to `Constants.DRIVE_FIELD_RELATIVE` for field-centric selection.
- `TeleOp` and `IvyTeleOp` square the stick inputs with `x -> Math.signum(x) * x * x`.
- Heading reset is usually mapped to `gamepad1.back`.
- `MecanumDrive` is the public drive-facing mechanism; the raw `Follower` is not the normal teleop entry point in team code.

## Rules

- Keep drive-input shaping in one place: `DriveVector` or the selected stick curve, not scattered sign flips across op modes.
- If field-centric math changes, verify both `DriveVector.rotateFieldCentric()` and the heading passed in from `MecanumDrive`.
- Preserve `periodic()` calling `follower.update()` exactly once per loop path.
- Prefer extending `MecanumDrive` APIs over having op modes reach inside `drivetrain` and `follower` directly.

## Common edit targets

- Change response curve or deadband behavior: `DriveVector.java`
- Change motor-power mapping or field-centric handling: `Drivetrain.java`
- Change public teleop-facing drive behavior or pose resets: `MecanumDrive.java`

## Validation ideas

- Check that forward stick still drives the robot in the intended direction.
- Check field-centric orientation after a heading reset.
- If motor-power math changed, use `MotorDirectionTest` or another safe low-speed test op mode before trusting match play.

