# Project Skills Index

This folder contains repo-specific skills for working on
`20373-Chimera-Biobuzz-26-27`.

## Skills

- `autonomous-path-patterns/` -- autonomous state machines, `Path`, and `PathChain` construction
- `build-and-validation/` -- TeamCode build, validation, and troubleshooting workflows
- `constants-not-configured/` -- intentionally provisional robot, field, and game constants
- `drivetrain-and-drivevector/` -- drive math, field-centric behavior, stick curves, and `DriveVector`
- `ftc-sdk-hardwaremap-and-lifecycle/` -- direct FTC SDK APIs, `HardwareMap`, and op mode lifecycle
- `ftc-sdk-opmodes/` -- FTC op mode creation, annotations, and hardware setup
- `gamepad-and-driver-input/` -- FTC gamepad APIs and the project's gamepad wrappers
- `ivy-command-patterns/` -- Pedro Ivy command composition and scheduling
- `library-utilities-and-shared-types/` -- lightweight reusable types and utilities under `library/`
- `limelight-wrapper-and-pipelines/` -- the Limelight wrapper and color, AprilTag, and detector pipelines
- `localization-and-vision/` -- localization, vision-assisted behavior, and related pose updates
- `mechanisms-and-hardware-wrappers/` -- mechanisms, hardware wrappers, and custom hardware devices
- `opmode-variants-and-pose-handoff/` -- op mode variants and `configs.Blackboard` pose/state handoff
- `pedro-localizers-and-pose-estimation/` -- Pedro localizer configuration and pose-estimation flow
- `pedro-pathing-and-ivy/` -- Pedro followers, paths, localizers, and Ivy integration
- `robot-configurations-and-follower-builder/` -- robot profiles, follower construction, and mechanism registration
- `skill-updates/` -- maintaining the skills index and skill guides from the current codebase
- `teamcode-library-architecture/` -- TeamCode architecture, hardware wrappers, mechanisms, and constants
- `tests-and-tuning-opmodes/` -- test op modes, tuning flows, and validation helpers

Choose the narrowest matching skill first; cross-cutting work can require more
than one guide.

## Current architecture notes

- `Constants.currentBot` selects the profile used by op-mode factories. A
  helper existing in `Constants` does **not** mean the selected profile
  registers that mechanism, so check `Constants.hasMechanism(...)` or use
  runtime-test guards such as `tests/ftcSdk/androidTest.java`'s
  `skipIfMechanismMissing(...)` before assuming a factory helper is available.
- `Constants.create...(...)` and `RobotConstants.createMechanism(...)` create
  only mechanisms registered by the active profile; the helpers throw when the
  requested mechanism is absent.
- `Constants.createMechanisms(hardwareMap)` is the aggregate factory for every
  profile-managed mechanism. The shared teleops currently require
  `RobotDrive.class` from that aggregate and treat `Intake` / `Limelight` as
  optional lookups.
- Profile-managed mechanisms now resolve nested builder-style params through
  `Mechanism.Params`, `Mechanism.resolveParams(...)`, and
  `RobotConstants.getMechanismParams(...)`.
- `RobotConstants.configureDriveMechanisms()` currently registers the public
  `RobotDrive` mechanism, while direct follower construction still lives on
  `RobotConstants.createFollower(...)`.
- `library/planners/` now contains the planner-style autonomous stack:
  `PathPlanner`, `PathPlannerList`, `PlanBuilder`, and
  `library/planners/paths/Utils.java`.
- SDK-registerable devices are separate from profile-managed mechanisms:
  `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator` are
  under `library/hardware/hardwareDevice/`.
- Validation is now split between driver-station test op modes under `tests/`
  / `tests/ftcSdk/` and JVM unit tests under `TeamCode/src/test/java`, with
  `UnitTestAll.java` acting as the single suite entry point.
- `Timer` and `OpModeHelper.dt` now use `double` time values, and
  `library/math/MathUtils.kt` also owns adaptive path-smoothing helpers used by
  planner code in addition to unit conversion, heading normalization, and
  FTC/Pedro pose conversion.
