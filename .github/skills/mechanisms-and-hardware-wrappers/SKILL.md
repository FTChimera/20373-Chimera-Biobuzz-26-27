---
name: mechanisms-and-hardware-wrappers
description: Use this skill when asked to add or modify TeamCode wrappers, hardware factories, or subsystem classes under `library/hardware` or `mechanisms`.
---

# Mechanisms and Hardware Wrappers

Use this skill when extending the team’s abstraction layers instead of writing raw FTC SDK calls directly in op modes.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Mechanisms.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Motor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`

## Design intent in this repo

1. Low-level device access belongs in wrappers such as `Motor` or `Limelight`.
2. Subsystem behavior belongs in mechanisms such as `Intake`, `Drivetrain`, and `MecanumDrive`.
3. Shared creation points belong in `Mechanisms`.
4. Op modes should orchestrate mechanisms, not become the main place for hardware-map lookups and device math.

## Existing patterns to reuse

- `Intake` wraps one `Motor` and exposes both direct methods and Ivy command helpers.
- `Drivetrain` owns the four drive `Motor` instances and applies `DriveVector` math.
- `MecanumDrive` combines a Pedro `Follower` with the team `Drivetrain` wrapper.
- `Mechanisms` exposes factory-style helpers like `Mechanisms.intake(hardwareMap)` and `Mechanisms.limelight(hardwareMap)`.

## Rules

- Add new reusable hardware access under `library/hardware/...` first.
- Add robot behavior under `mechanisms/...` second.
- Prefer returning wrapper objects from `Mechanisms` instead of constructing them repeatedly in every op mode.
- Preserve chainable convenience methods where a wrapper already uses them, as `Motor` does.
- If a mechanism already exposes an Ivy command helper, prefer extending that API instead of duplicating inline commands in op modes.

## Good edits for common tasks

- New motor behavior like brake/float/PIDF convenience: `Motor.java`
- New intake-level operation: `Intake.java`
- New multi-subsystem factory: `Mechanisms.java`
- New drive behavior or teleop helper: `Drivetrain.java` or `MecanumDrive.java`

## Things to avoid

- Bypassing `Motor` with raw `DcMotorEx` in an op mode unless the wrapper itself needs improvement.
- Bypassing `Limelight` with raw `Limelight3A` calls all over the codebase.
- Stuffing subsystem state machines into `Mechanisms.java`; it should stay a lightweight factory surface.

