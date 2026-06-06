---
name: teamcode-library-architecture
description: Use this skill when asked to work with the team's current library, hardware wrappers, mechanisms, constants, or overall TeamCode architecture instead of generic FTC examples.
---

# TeamCode Library Architecture

Use this skill to stay aligned with the repo’s current architecture instead of falling back to generic FTC examples.

## Package layout

- `library/`: small reusable types and hardware wrappers such as `DriveVector`, `Timer`, `AllianceColor`, `Motor`, `Direction`, `Polynomial`, gamepad helpers, and webcam helpers.
- `mechanisms/`: robot subsystems plus the shared abstract base class `Mechanism`.
- `opModes/`: shared base op mode logic.
- `opModes/variants/`: concrete driver-station-visible wrappers with FTC annotations.
- `pedroPathing/`: Pedro-specific helpers, tuning entry points, adapters, and custom geometry such as `custom/EquationCurve`.
- `library/robot/`: active robot-profile and follower-builder configuration such as `RobotConstants` and `RobotConfigurations`.
- `Constants.java`: the active robot profile, default poses, and op-mode-facing convenience factories.
- `Blackboard.java`: static cross-opmode state such as pose handoff.

## House style for mechanisms and wrappers

This repo now wants reusable subsystems and higher-level wrappers to share one construction shape instead of ad hoc constructors in op modes.

Use this pattern:

1. Reusable robot subsystems and wrappers that depend on the active robot profile should extend `mechanisms/Mechanism`.
2. Give them a constructor shaped like `(HardwareMap, RobotConstants)`.
3. Use the `bot` profile passed into that constructor for shared robot-profile data and mechanism enablement checks.
4. Register optional subsystems in each robot preset with `RobotConstants.addMechanism(...)`.
5. Instantiate from `RobotConstants.createMechanism(...)`, usually through a small convenience helper in `Constants` such as `createIntake(hardwareMap)`.
6. Op modes should call the existing `Constants.create...(...)` helpers instead of using direct constructors.

Some mechanisms still own nested params types such as `Intake.INTAKE_PARAMS` or `Limelight.LIMELIGHT_PARAMS`, but the uniform repo-level construction pattern is now based on `Mechanism` plus `RobotConstants.createMechanism(...)`.

## Where configuration belongs

- Put drive motor names, `Direction`-based drive directions, Pinpoint data, follower tuning, path constraints, and the enabled-mechanism list in `library/robot/RobotConstants` / `RobotConfigurations`.
- Put default poses and small top-level convenience factories in `Constants`.
- Keep mechanism-specific behavior and optional nested params types in the mechanism or wrapper class that owns them.
- Keep lightweight math helpers such as `Polynomial` in `library/` and Pedro-specific curve integration such as `EquationCurve` in `pedroPathing/custom/`.
- Do not turn `RobotConstants` into a catch-all for arbitrary per-mechanism tuning unless that data is truly robot-profile-wide.

## Existing wrappers and mechanisms to reuse

- `library.hardware.actuators.Direction`
- `library.hardware.actuators.Motor`
- `library.Polynomial`
- `library.hardware.webcam.Limelight`
- `library.DriveVector`
- `mechanisms.Intake`
- `mechanisms.Drivetrain`
- `mechanisms.MecanumDrive`
- `mechanisms.Mechanism`
- `library.robot.RobotConstants`
- `library.robot.RobotConfigurations`
- `pedroPathing.custom.EquationCurve`

## Rules

- Prefer extending existing wrappers over bypassing them.
- When adding a new profile-managed mechanism, extend `Mechanism`, use the `(HardwareMap, RobotConstants)` constructor shape, and register it in `RobotConfigurations`.
- Add a `Constants.create...(...)` helper when that mechanism is expected to be used directly by op modes or other high-level code.
- Keep op modes focused on orchestration, not low-level hardware construction.
- Keep pose transfer through `Blackboard.pose` if autonomous and teleop need to share localization state.
- Match the repo’s split between mechanism logic, wrapper logic, and op mode orchestration.

## Good places to edit for common tasks

- New top-level subsystem convenience factory or active-profile selection: `Constants.java`
- New drive profile or Pedro follower constant set: `library/robot/...`
- New simple hardware wrapper: `library/hardware/...`
- New subsystem behavior: `mechanisms/...`
- New teleop or autonomous entry: `opModes/variants/...`
- New Pedro/localization behavior: `pedroPathing/...`
- New lightweight math helper: `library/...`

## Things not to assume

- Generic FTC sample patterns from `FtcRobotController/external/samples/` are not the house style for TeamCode.
- New mechanisms should not be instantiated directly from op modes when a `Constants.create...(...)` factory belongs there.
- A mechanism no longer needs a centrally shared params singleton in `Constants` to fit repo style; the important uniformity point is the `Mechanism` + `RobotConstants.createMechanism(...)` construction flow.
- `Polynomial` is a repo utility class, while `EquationCurve` is the Pedro-specific adapter that turns polynomial geometry into a `CustomCurve`.

## Current build-tooling note

- `TeamCode/build.gradle` now wires Project Lombok annotation processing and Sloth load / runtime support, so architecture or cleanup work should preserve that module-level tooling unless the codebase is intentionally being migrated away from it.

