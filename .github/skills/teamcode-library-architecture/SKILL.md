---
name: teamcode-library-architecture
description: Use this skill when asked to work with the team's current library, hardware wrappers, mechanisms, constants, or overall TeamCode architecture instead of generic FTC examples.
---

# TeamCode Library Architecture

Use this skill to stay aligned with the repo’s current architecture instead of falling back to generic FTC examples.

## Package layout

- `library/`: small reusable types and hardware wrappers such as `DriveVector`, `Timer`, `AllianceColor`, `Motor`, `Direction`, gamepad helpers, and webcam helpers.
- `mechanisms/`: robot subsystems such as `Intake`, `Drivetrain`, and `MecanumDrive`.
- `opModes/`: shared base op mode logic.
- `opModes/variants/`: concrete driver-station-visible wrappers with FTC annotations.
- `pedroPathing/`: Pedro-specific helpers, tuning entry points, and adapters.
- `library/robot/`: active robot-profile and follower-builder configuration such as `RobotConstants` and `RobotConfigurations`.
- `Constants.java`: the central factory/config surface for editable subsystem params, default poses, and the active robot profile.
- `Blackboard.java`: static cross-opmode state such as pose handoff.

## House style for mechanisms and wrappers

This repo now wants new mechanisms and higher-level wrappers created through `Constants`, not ad hoc constructors in op modes.

Use this pattern:

1. A mechanism or wrapper owns its own nested params type, such as `Intake.INTAKE_PARAMS` or `Limelight.LIMELIGHT_PARAMS`.
2. The mechanism stores a params instance on the object, for example `public final INTAKE_PARAMS params;`.
3. The shared editable params instance lives centrally in `Constants`, for example `public static final Intake.INTAKE_PARAMS INTAKE_PARAMS = new Intake.INTAKE_PARAMS();`.
4. `Constants` exposes a factory such as `createIntake(hardwareMap)` that passes the central params instance into the constructor.
5. Op modes call `Constants.create...(...)` instead of `new Mechanism(...)` directly.

This centralization is important because `Constants` is the place the team wants to expose Dashboard-editable values and panels.

## Where configuration belongs

- Put non-drive subsystem params in the subsystem or wrapper class, but keep the shared instance in `Constants`.
- Put drive motor names, `Direction`-based drive directions, Pinpoint data, follower tuning, and path constraints in `library/robot/RobotConstants` / `RobotConfigurations`.
- Put default poses and cross-mechanism toggles in `Constants`.
- Do not move non-drive mechanism params into `RobotConstants`; that class is for robot-profile and Pedro drive configuration.

## Existing wrappers and mechanisms to reuse

- `library.hardware.actuators.Direction`
- `library.hardware.actuators.Motor`
- `library.hardware.webcam.Limelight`
- `library.DriveVector`
- `mechanisms.Intake`
- `mechanisms.Drivetrain`
- `mechanisms.MecanumDrive`
- `library.robot.RobotConstants`
- `library.robot.RobotConfigurations`

## Rules

- Prefer extending existing wrappers over bypassing them.
- When adding a new mechanism, also add a central params instance and factory in `Constants`.
- Keep op modes focused on orchestration, not low-level hardware construction.
- Keep pose transfer through `Blackboard.pose` if autonomous and teleop need to share localization state.
- Match the repo’s split between mechanism logic, wrapper logic, and op mode orchestration.

## Good places to edit for common tasks

- New non-drive mechanism params or factory: `Constants.java`
- New drive profile or Pedro follower constant set: `library/robot/...`
- New simple hardware wrapper: `library/hardware/...`
- New subsystem behavior: `mechanisms/...`
- New teleop or autonomous entry: `opModes/variants/...`
- New Pedro/localization behavior: `pedroPathing/...`

## Things not to assume

- Generic FTC sample patterns from `FtcRobotController/external/samples/` are not the house style for TeamCode.
- New mechanisms should not be instantiated directly from op modes when a `Constants.create...(...)` factory belongs there.
- The Dashboard-editable params instance should live centrally in `Constants`, not as a second static params object hidden inside each mechanism.

## Current build-tooling note

- `TeamCode/build.gradle` now wires Project Lombok annotation processing and Sloth load / runtime support, so architecture or cleanup work should preserve that module-level tooling unless the codebase is intentionally being migrated away from it.

