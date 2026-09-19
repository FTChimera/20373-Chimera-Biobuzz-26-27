# Project Skills Index

This index describes the repository-specific guidance for
`20373-Chimera-Biobuzz-26-27`.

## Using this index

Choose the narrowest skill that owns the requested change.

Read the matching `SKILL.md` before changing code.

Use a second skill when a change crosses an explicit boundary.

For example, a competition autonomous can need the autonomous, planning,
robot-configuration, and validation skills.

Source code, not an old guide or compiled output, is authoritative.

Check `TeamCode/src/main/java` and `TeamCode/src/test/java` before documenting
an API or workflow.

## Skills

- `autonomous-path-patterns/` — autonomous state machines and Pedro paths.
- `build-and-validation/` — Gradle builds, JVM tests, lint, and FTC checks.
- `constants-not-configured/` — intentional robot, field, and game placeholders.
- `drivetrain-and-drivevector/` — teleop drive math and field-centric behavior.
- `ftc-sdk-hardwaremap-and-lifecycle/` — FTC lifecycle and `HardwareMap` APIs.
- `ftc-sdk-opmodes/` — FTC annotations and concrete op mode entries.
- `gamepad-and-driver-input/` — `GamepadEx`, `DualGamepad`, and bindings.
- `ivy-command-patterns/` — Ivy command composition and scheduler ownership.
- `library-utilities-and-shared-types/` — reusable library types and helpers.
- `limelight-wrapper-and-pipelines/` — Limelight wrapper and pipeline selection.
- `localization-and-vision/` — Pinpoint, AprilTags, and vision-assisted behavior.
- `mechanisms-and-hardware-wrappers/` — mechanisms and hardware wrappers.
- `opmode-variants-and-pose-handoff/` — Blackboard configuration and handoff.
- `pedro-localizers-and-pose-estimation/` — Pedro localization setup.
- `pedro-pathing-and-ivy/` — followers, paths, and Ivy integration.
- `robot-configurations-and-follower-builder/` — profiles and follower creation.
- `skill-updates/` — maintaining skills and the FTC development guide.
- `teamcode-library-architecture/` — current TeamCode architecture.
- `tests-and-tuning-opmodes/` — test, tuning, and validation op modes.

## Active match entries

The concrete competition teleop entry is `tele/TeleOp.java`.

It is a `LinearOpMode` with nested `Red_Tele` and `Blue_Tele` entries.

It creates `DualGamepad` from both FTC gamepads.

It creates `State(RobotMode.TELEOP)` and a teleop `Robot`.

It restores alliance and pose from `Blackboard` when available.

It starts teleop drive before the match begins.

Each active loop updates both gamepads and calls `robot.periodic(robotState)`.

It stops the aggregate robot when the linear op mode exits.

The concrete competition autonomous entry is `autos/Garden8.java`.

It is an iterative `OpMode` with nested `redalliance` and `bluealliance`
entries.

`init()` resets Ivy, creates the robot, and builds the Garden plan.

`start()` starts the aggregate robot.

`loop()` executes Ivy and handles the autonomous time limit and parking logic.

`stop()` stops the robot if needed and resets Ivy.

Garden 8 builds its match sequence with:

```java
PlanBuilder.startGardenPose(robot)
        .shootPreloads()
        .intakeGarden()
        .schedule();
```

## Robot and mechanism lifecycle

Every profile-managed mechanism extends `mechanisms/Mechanism`.

The required lifecycle surface is:

```java
public abstract void periodic(State robotState);
public abstract void stop();
```

Do not document or implement a mode argument on `periodic`.

`State` carries the mode and the aggregate fills in robot, alliance/mode, and
`OpModeHelper` before dispatch.

`Robot` is itself a mechanism aggregate.

Its constructor creates every mechanism registered by the active profile.

Use `get(...)` for optional mechanisms and `require(...)` for prerequisites.

`Robot.periodic(state)` validates the state mode, updates the helper, populates
the state, and dispatches to every registered mechanism.

Call `robot.start()` before normal periodic work.

`Robot.stop()` stops all mechanisms, closes the helper, and invokes
`saveEndVals()`.

For autonomous, `saveEndVals()` stores a velocity-compensated pose and current
alliance in `Blackboard` after at least one periodic update.

For teleop, it resets Blackboard values except the selected driver preference.

`Robot.forceRelocalize()` is explicit; vision periodic updates do not silently
overwrite the drive pose.

## Profiles and factories

`Constants.currentBot` is a compile-time assignment in `Constants.java`.

The checked-in assignment is `RobotConfigurations.FirstRobot()`.

Changing profiles requires editing `Constants.java` and rebuilding.

It is not a freely runtime-selectable setting.

`Constants.createRobot(hardwareMap, mode)` is the usual shared-flow factory.

`RobotConstants.createMechanism(...)` creates only registered mechanisms.

`configureDriveMechanisms()` registers `RobotDrive`.

Other mechanisms need explicit profile registration.

`FirstRobot()` contains placeholder zero values for velocity, heading PIDF,
and centripetal scaling; tune these before treating the profile as complete.

`FieldUpdates.getGoalPose()` currently returns `null` and is a scaffold, not
a usable goal-pose source.

## Planning and vision intake

`library/planners/PlanBuilder` composes supported planner steps.

Its current Garden entry point is `startGardenPose(robot)`.

The builder owns a `PathPlannerList`, and `schedule()` delegates to that list.

`Common.init(alliance)` initializes alliance-aware planner poses; `Robot`
calls it while constructing the aggregate.

The current builder supports Garden preload shooting, Garden intake, explicit
Pedro paths, linear paths, waits, and a deprecated `intakeWVision(...)` helper.

Loading-side planner branches are not complete and should not be presented as
available match behavior.

`IntakeBallPath` generates detector-based collection paths from an explicit
follower, robot pose, Vision mechanism, requested count, and detections.

It filters, scores, de-duplicates, orders, and then smooths candidates.

It can choose Pedro curve-through geometry or adaptive Bezier handles.

The no-argument `generateIntakePathFromCam()` overload returns an empty
`PathChain`; it is a scaffold, not camera integration.

## Input and drive

`DualGamepad` wraps FTC `gamepad1` and `gamepad2` as two `GamepadEx` objects.

Call `drivers.update()` once per loop before consuming edges, sticks, or
bindings.

`drivers.result()` returns a `GpBindings` view for the current input surfaces.

The competition teleop assigns that result to its `State`.

`RobotDrive` reads configured bindings and delegates drive behavior to
`MecanumDrive`.

Keep stick shaping and field-centric transforms in the existing drive layers,
not copied into an op mode.

## Shared library

`library/math/MathUtils.java` is plain Java.

It provides distance and heading helpers, unit conversion, heading
normalization, FTC/Pedro pose conversion, and smoothing helpers for intake
paths.

`library/control/` contains the reusable controller types.

`Timer` and `OpModeHelper.dt` use `double` time values.

## Validation

Use the existing Gradle tasks from the repository root:

```zsh
./gradlew :TeamCode:assembleDebug --console=plain
./gradlew :TeamCode:lint --console=plain
./gradlew :TeamCode:testDebugUnitTest --console=plain
```

`UnitTestAll.java` is the JVM suite entry point.

Driver-station validation lives under `tests/` and `tests/ftcSdk/`.

Pedro tuner selection lives in `pedroPathing/Tuning.java`.

PID relay-feedback tuning op modes live in `pid/autoTune/`.

Use a focused factory or hardware test after a build when changing physical
configuration.

## Documentation maintenance

The development hub is `ftcdev-20373.html`.

The chapter index is `ftcdev-20373/index.html`.

The numbered HTML chapters are companion documentation, not generated source.

When updating these docs, preserve the hub and chapter navigation links.

Search all guide files for deleted paths and removed symbols before finishing.

Do not use `TeamCode/bin/` as a source of API truth.

Run `git diff --check` and a focused stale-reference search after editing.
