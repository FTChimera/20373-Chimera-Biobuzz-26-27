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

When a docs-maintenance request says to update "all the skill files", treat that
as updating every repo document that acts as a reference for anyone or anything
modifying the codebase: this index, the relevant `SKILL.md` files, and
the `ftcdev-20373.html` landing hub plus its long-form static chapters under
`ftcdev-20373/`.

## External dependency lookup shortcuts

When TeamCode behavior comes from an external dependency instead of repo-local
code, inspect the downloaded module under
`~/.gradle/caches/modules-2/files-2.1/` using the version pinned in
`build.dependencies.gradle` before guessing from memory.

- `com.pedropathing/core` and `com.pedropathing/ftc` for Pedro Pathing motion,
  path, and follower internals
- `com.pedropathing/ivy` for Ivy command, scheduler, and builder behavior
- `com.pedropathing/telemetry` for Pedro telemetry wiring and
  `pedroPathing/Tuning.java` op-mode selection issues
- `com.bylazar/fullpanels` and sibling `com.bylazar/...` modules for
  panels-related code
- `org.firstinspires.ftc/RobotCore`, `org.firstinspires.ftc/Hardware`, and,
  when vision APIs are involved, `org.firstinspires.ftc/Vision` for FTC SDK
  implementation details

## Current architecture notes

- `Constants.currentBot` selects the profile used by op-mode factories. A
  helper existing in `Constants` does **not** mean the selected profile
  registers that mechanism, so check `Constants.hasMechanism(...)` or use
  runtime-test guards such as `tests/ftcSdk/androidTest.java`'s
  `skipIfMechanismMissing(...)` before assuming a factory helper is available.
- `Constants.create...(...)` and `RobotConstants.createMechanism(...)` create
  only profile-managed mechanisms registered by the active profile. Camera
  behavior is exposed through the profile-managed `Vision` mechanism.
- `Constants.createRobot(hardwareMap)` is the aggregate factory for every
  profile-managed mechanism, including `Vision`. The aggregate updates Vision;
  its owning flow explicitly starts and stops the selected pipeline.
- Use `Constants.createRobot(hardwareMap, RobotMode.TELEOP)` or
  `Constants.createRobot(hardwareMap, RobotMode.AUTONOMOUS)` for new shared
  flows, create the matching `State`, and call `robot.start()` before the first
  `robot.periodic(...)`. `Robot.stop()` owns autonomous Blackboard handoff.
- `Robot.build()` returns `RobotIvyBuilder`, whose commands cover robot
  lifecycle, periodic updates, follower paths, intake, drive, vision, and
  explicit `forceRelocalize()` actions.
- `Vision.periodic(...)` refreshes orientation and cached results only. It does
  not overwrite the drive pose; call `Robot.forceRelocalize()` or schedule the
  builder equivalent when a valid MT2 pose should be applied.
- Profile-managed mechanisms now resolve nested builder-style params through
  `Mechanism.Params`, `Mechanism.resolveParams(...)`, and
  `RobotConstants.getMechanismParams(...)`.
- `RobotConstants.configureDriveMechanisms()` currently registers the public
  `RobotDrive` mechanism, while direct follower construction still lives on
  `RobotConstants.createFollower(...)`.
- `library/planners/` contains the planner-style autonomous stack:
  `PathPlanner`, `PathPlannerList`, `PlanBuilder`, and
  `library/planners/paths/Utils.java`; `tests/PlannerIntakeVisionAuto.java`
  exercises its detector-driven intake and return-path flow.
- `library/game/Field.java` owns JTS-backed field polygons, robot-footprint
  sweeps, and `ContactState` classification for preseason geometry work.
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
- `library/game/RobotMode` and `State` distinguish autonomous from teleop
  behavior. `PIDFController` validates non-null, equal-dimension state/target
  inputs and skips derivative output on its first update; `LinearInterp`
  requires configured, ascending keys; and `SlidesMotor` rejects invalid
  limits, positions, and percentages.
