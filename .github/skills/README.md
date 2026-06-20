# Project Skills Index

This folder contains repo-specific skills for working on `20373-Chimera-Biobuzz-26-27`.

## Existing broad skills

- `build-and-validation/`
- `ftc-sdk-opmodes/`
- `localization-and-vision/`
- `pedro-pathing-and-ivy/`
- `teamcode-library-architecture/`

## Additional focused skills

- `robot-configurations-and-follower-builder/` — robot profiles, `RobotConstants`, `RobotConfigurations`, `addMechanism(...)`, and Pedro follower / mechanism creation
- `mechanisms-and-hardware-wrappers/` — wrapper and subsystem patterns in `library/hardware` and `mechanisms`
- `drivetrain-and-drivevector/` — drive math, `DriveVector`, `Drivetrain`, and `MecanumDrive`
- `opmode-variants-and-pose-handoff/` — shared op mode bases, variants, and `configs.Blackboard` pose / driver-state handoff
- `ivy-command-patterns/` — Pedro Ivy scheduling, command construction, and `CommandConverter`
- `ftc-sdk-hardwaremap-and-lifecycle/` — FTC SDK API usage, `HardwareMap`, annotations, and lifecycle rules
- `limelight-wrapper-and-pipelines/` — Limelight wrapper usage and pipeline conventions
- `autonomous-path-patterns/` — state-machine, `PathChain`, and custom-curve patterns used in autonomous
- `gamepad-and-driver-input/` — raw FTC gamepad APIs plus `GamepadEx` / `DualGamepad` wrapper conventions
- `tests-and-tuning-opmodes/` — test op modes, Pedro tuning, and validation habits

## Notes

Most skills are intentionally narrow and repo-specific. Prefer the focused skill when the task clearly matches one of these areas, and use the broader skills for cross-cutting changes.

Across the skill set, the current house style is:

- put drive/profile data in `library/robot/RobotConstants` and `RobotConfigurations`
- register optional subsystems per robot profile with `RobotConstants.configureDriveMechanisms()` and `RobotConstants.addMechanism(...)`
- have reusable subsystems and wrappers extend `mechanisms/Mechanism`
- give profile-managed subsystems a constructor shaped like `(HardwareMap, RobotConstants)` so `RobotConstants.createMechanism(...)` can instantiate them uniformly
- use `Constants.create...(...)` helpers as the normal op-mode-facing convenience entry points when a subsystem is used broadly

Current repo details worth remembering while using these skills:

- drive motor directions are represented with `library/hardware/actuators/Direction.java`, then converted to FTC SDK directions at the wrapper/profile boundary
- `TeamCode/build.gradle` wires Project Lombok annotation processing and the Sloth load / hot-reload tooling, so build troubleshooting should account for both
- `tests/MecanumTeleOp.java` is a first-class iterative test op mode for robot-centric drive, field-centric drive, IMU yaw reset, and manual motor-direction checks
- `library/math/Polynomial.java` and `pedroPathing/custom/EquationCurve.java` are now part of the custom pathing toolbox when Bezier geometry is not the right fit

