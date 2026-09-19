---
name: teamcode-library-architecture
description: Use this skill for the current TeamCode architecture, factories, mechanisms, and match code boundaries.
---

# TeamCode Library Architecture

## Current package map

- `autos/`: competition autonomous entries; `Garden8.java` (`OpMode`) is the
  current planner consumer, with nested `redalliance`/`bluealliance`
  `@Autonomous` subclasses.
- `tele/`: match teleop; `TeleOp.java` (`LinearOpMode`) is the current
  annotated Main TeleOp, with nested `Red_Tele`/`Blue_Tele` `@TeleOp`
  subclasses.
- `configs/`: `Blackboard.java` (static cross-op-mode state) and
  `ConfigOpMode.java` (selectable pre-start configuration menu).
- `mechanisms/`: profile-managed behavior — `Mechanism` (abstract base),
  `Robot` (aggregate), `RobotDrive`, `Intake`, `Vision`, `Shooter`, and
  `FieldUpdates`.
- `library/`: reusable code, split into `control/` (PIDF/bang-bang/composite
  controllers), `filters/` (e.g. low-pass), `game/` (`State`, `RobotMode`,
  `Field`, `HiveState`, `Pollen`), `hardware/` (`actuators/`, `gamepad/`,
  `hardwareDevice/`, `webcam/`; `leds/` is an empty leftover directory),
  `interpolation/`, `math/` (`MathUtils`, `EquationCurve` support),
  `planners/` (`Common`, `PlanBuilder`, `PathPlanner`, `PathPlannerList`,
  `paths/`), and `robot/` (`RobotConstants`, `RobotConfigurations`,
  `RobotIvyBuilder`, `Driver`, `drivetrain/MecanumDrive`).
- `pedroPathing/`: `Tuning.java` (Pedro tuning menu), `AlliancePoses.java`,
  the renamed `Pedro.java` helper, and `custom/` adapters
  (`LimelightPinpointLocalizer`, `EquationCurve`).
- `pid/autoTune/`: relay-feedback AutoTune op modes and `TuningConstants`,
  plus `master/` (`PIDMaster`, `DualMotorPIDMaster`) shared logic.
- `tests/`: Driver Station validation op modes, including `tests/ftcSdk/` for
  the profile-factory smoke tests built on `androidTest`.
- `TeamCode/src/test/java`: JVM/host-side JUnit tests, aggregated by
  `UnitTestAll`.

`opModes/` and its Ivy-specific bases/variants were removed from
`TeamCode/src`; only the stale `TeamCode/bin/` build output (not source) still
contains that directory, and it must not be treated as current architecture.

## Construction and lifecycle

The standard flow is:

```java
Robot robot = Constants.createRobot(hardwareMap, RobotMode.TELEOP);
State state = new State(RobotMode.TELEOP);
// after FTC start
robot.start();
robot.periodic(state);
robot.stop();
```

Each registered profile mechanism has a `(HardwareMap, RobotConstants)`
constructor, implements `periodic(State)` and `stop()`, and uses copied nested
`Mechanism.Params` when it needs profile-specific tuning. `Robot` (itself a
`Mechanism`) aggregates the profile's mechanism set in class-name order,
enriches `State` each cycle (`state.setRobot(this)`, alliance/mode, the
`OpModeHelper`), keeps bulk-cache/battery reads current through
`OpModeHelper`, and owns shutdown/persistence via `stop(boolean saveEndVals)`
-> `saveEndVals()`.

## Boundaries

- `RobotConstants` / `RobotConfigurations`: robot-specific hardware and
  follower configuration plus mechanism registration (`Mechanisms` set and
  `mechanismParams` overrides). See `robot-configurations-and-follower-builder`.
- mechanism params (`XxxMechanism.XXX_PARAMS`): local behavior and optional
  per-profile overrides, always copied through `Params.copy()`.
- `library/hardware/hardwareDevice`: direct SDK device types
  (`SlidesMotor`, `RevPotentiometer`, `RGBIndicator`), not aggregate
  mechanisms — construct these directly, not through `Robot`.
- `library/planners`: reusable, stateful autonomous modules
  (`PlanBuilder`/`PathPlanner`/`PathPlannerList`/`paths/*`). See
  `pedro-pathing-and-ivy`.
- `library/game`: reusable geometry, game types (`Field`, `HiveState`,
  `Pollen`), `State`, and `RobotMode`; not finished strategy by default (e.g.
  `FieldUpdates.getGoalPose()` still returns `null`).

Use `Vision` for new camera work (see `limelight-wrapper-and-pipelines`) and
`RobotIvyBuilder` for aggregate Ivy commands. Use `PlanBuilder` and
`autos/Garden8` for current match autonomous structure. Do not treat
unfinished `Shooter`, `FieldUpdates`, `ShotSolver`, `HiveState`, or `Common`'s
placeholder `AlliancePoses` (`new Pose()` field values) as calibrated match
behavior — they compile and run but have not been measured for the current
field/robot.

## Where cross-cutting Ivy code lives

Ivy `Command`/`Scheduler` usage spans three layers rather than one package:
`library/robot/RobotIvyBuilder` wraps aggregate `Robot` actions,
`library/planners/PathPlanner`/`PathPlannerList` compose planner-level
commands (`asCommand()`, `fromPrev(...)`), and `pedroPathing/Pedro` provides
low-level Pedro-specific command/path helpers (`runIvy`, `ivyFollowPath`,
`linear`). Mechanism-level Ivy commands (e.g. `Intake.fullPower()`,
`MecanumDrive.teleOp(...)`) live directly on the mechanism/drivetrain class
that owns the underlying state, not in a separate command package — there is
no `commands/` directory in current source.

## Learning/scratch code is not match code

`learning/` (`flyset`, `kotlinTest`, `teachEason`) holds exploratory or
teaching snippets outside the `autos`/`tele`/`mechanisms`/`library` boundaries
described above. Do not treat classes under `learning/` as part of the current
architecture when documenting match behavior, and do not assume they compile
against the latest `Mechanism`/`RobotConstants` contracts.
