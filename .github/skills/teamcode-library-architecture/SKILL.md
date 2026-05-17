---
name: teamcode-library-architecture
description: Use this skill when asked to work with the team's current library, hardware wrappers, mechanisms, constants, or overall TeamCode architecture instead of generic FTC examples.
---

# TeamCode Library Architecture

Use this skill to stay aligned with the current repo structure and naming patterns.

## Package layout

- `library/`: small reusable types and wrappers such as `DriveVector`, `Timer`, `AllianceColor`, `Motor`, gamepad helpers, and webcam helpers.
- `mechanisms/`: robot subsystems such as `Drivetrain`, `MecanumDrive`, and `Intake`.
- `opModes/`: shared op mode logic.
- `opModes/variants/`: annotated concrete teleop/autonomous entries.
- `pedroPathing/`: Pedro-specific helpers, tuning, and adapters.
- `library/robot/`: robot profiles and follower-builder configuration such as `RobotConstants` and `RobotConfigurations`.
- `Constants.java`: thin global constants surface for non-drive names, default poses, and active robot-profile selection.
- `Blackboard.java`: static cross-opmode state.

## Current design intent

1. Low-level hardware access belongs in wrappers under `library/hardware/`.
2. Robot behavior belongs in mechanisms under `mechanisms/`.
3. Op modes orchestrate mechanisms; they should not become the main place for motor math or camera APIs.
4. Shared configuration belongs in `Constants` or `library/robot/`, depending on whether it is a global constant or an active robot profile.
5. Reusable alliance-agnostic logic belongs in shared base classes, with variants providing annotations and alliance choice.

## Existing wrappers to reuse

- `library.hardware.actuators.Motor`
- `library.hardware.webcam.Limelight`
- `library.DriveVector`
- `mechanisms.Intake`
- `mechanisms.Drivetrain`
- `mechanisms.MecanumDrive`
- `library.robot.RobotConstants`
- `library.robot.RobotConfigurations`

## Rules

- Prefer extending the existing wrappers over bypassing them.
- Preserve existing naming unless you are cleaning up the whole abstraction you touched.
- When adding drive hardware configuration, update `library/robot/RobotConstants` and `RobotConfigurations`; keep non-drive globals in `Constants`.
- Keep pose transfer through `Blackboard.pose` if autonomous and teleop need to share localization state.
- Match the repo's split between mechanism logic and op mode orchestration.

## Good places to edit for common tasks

- New non-drive hardware constant: `Constants.java`
- New drive profile or Pedro follower constant set: `library/robot/...`
- New simple actuator wrapper: `library/hardware/...`
- New subsystem behavior: `mechanisms/...`
- New teleop or autonomous entry: `opModes/variants/...`
- New Pedro/localization behavior: `pedroPathing/...`

## Things not to assume

- Generic FTC sample patterns from `FtcRobotController/external/samples/` are not the house style for TeamCode.
- Tuning values in `Constants` are not fully populated yet.
- The library is intentionally lightweight; prefer small extensions over introducing a large framework.
