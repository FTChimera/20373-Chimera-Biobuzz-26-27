---
name: opmode-variants-and-pose-handoff
description: Use this skill for selectable match entries, alliance selection, and Blackboard handoff.
---

# Op Mode Entries and Pose Handoff

The historical abstract-base plus `opModes/variants/` pattern no longer
exists in source (it only still appears under the stale `TeamCode/bin/`
compiled-classes output, which is not source). `opModes/`, `IvyAuton`,
`IvyTeleOp`, and separate `Blue_TeleOp` / `Red_TeleOp` files were removed from
`TeamCode/src`.

## Current selectable entries

- `tele/TeleOp.java` is the concrete `LinearOpMode` annotated
  `@TeleOp(name = Constants.TELEOP_NAME, group = Constants.COMPETITION_GROUP)`
  ("Main TeleOp"). Its nested static classes `Red_Tele` and `Blue_Tele` are
  separately annotated `@TeleOp(name = "Red TeleOp")` / `@TeleOp(name = "Blue
  TeleOp")`; each overrides `runOpMode()` to call the protected
  `setAllianceColor(AllianceColor.RED/BLUE)` (which writes directly to
  `Blackboard.Caption.ALLIANCE`) before delegating to `super.runOpMode()`.
  These exist as an alliance-selection fallback in case the Driver Station's
  autonomous-preselect flow is unavailable — prefer the Garden8 autonomous
  path for normal alliance selection.
- `autos/Garden8.java` is the concrete `OpMode`-based autonomous. Its nested
  `redalliance` and `bluealliance` static subclasses each run a **static
  initializer block** (`static { setAllianceColor(...); }`) that writes
  Blackboard alliance at class-load time (before `init()` runs), and are
  annotated `@Autonomous(name = "Red/Blue Garden 8", group =
  Constants.COMPETITION_GROUP, preselectTeleOp = Constants.TELEOP_NAME)` so the
  Driver Station auto-selects Main TeleOp afterward. Both nested classes share
  the same `PlanBuilder`-driven Garden planner implementation from the parent
  class's `init()`.
- `configs/ConfigOpMode.java` is the selectable pre-start configuration menu.

Use a compact nested class (annotated concrete `@TeleOp`/`@Autonomous`, or a
static-initializer alliance variant) only when it expresses real selection
data, as the two examples above do. Do not reintroduce an abstract-base plus
`variants/` package layout merely to reproduce the deleted arrangement.

## Blackboard structure

`configs/Blackboard.java` holds a static `List<Item>` seeded with one `Item`
per `Caption`: `POSE` (`Pose.class`), `DRIVER` (`Driver.class`), and `ALLIANCE`
(`AllianceColor.class`). `Blackboard.Item` enforces the caption's declared
class on both construction and `setValue(...)` (throwing
`IllegalArgumentException` on a mismatch), and remembers the **first** value
ever set as `defaultV`; `Item.reset()` restores that default rather than
clearing to `null`. A freshly constructed `Item(Caption)` with no value has
`val == null` and `defaultV == null` — `getValue()` on it returns `null`, which
callers must distinguish from a meaningful zero pose.
`Blackboard.get(Caption)`/`put(Caption, Object)`/`put(Item)`/`getAll()`/
`resetAll()` are the only entry points; there is no per-op-mode Blackboard
instance.

## Handoff owner: `Robot.saveEndVals()`

`Robot.saveEndVals()` is the single handoff owner and branches on
`getRobotMode()`:

- **Autonomous**: only runs if `lastState != null` (i.e. `Robot.periodic(state)`
  executed at least once — `Robot.periodic(...)` sets `lastState = state` at
  the top of every call). It writes `Blackboard.Caption.POSE` from
  `getPoseCompensatedForVelocity()` and, if `lastState.getAllianceColor() !=
  null`, writes `Blackboard.Caption.ALLIANCE` too.
- **Teleop**: resets every `Blackboard.Item` **except** `DRIVER` back to its
  default via `Item.reset()`, so a driver preference persists across a match
  but pose/alliance do not leak into the next op mode.

`Robot.stop(boolean saveEndVals)` calls `saveEndVals()` after every mechanism's
`stop()` and `helper.close()` when the flag is `true`; `Garden8.loop()` calls
`robot.stop(false)` first (to let the robot decelerate) and only calls
`robot.saveEndVals()` once velocity has settled, then `requestOpModeStop()`.

## Consuming the handoff in TeleOp

`TeleOp.getStartingPoseAndSetAlliance()` runs before `waitForStart()`: it
first adopts `Blackboard.Caption.ALLIANCE`'s value if the field-local
`allianceColor` is still `null`, then returns the Blackboard pose if one is
present and non-null, else falls back to `Constants.RED_DEFAULT_START_POSE` /
`Constants.BLUE_DEFAULT_START_POSE` keyed off the resolved alliance (defaulting
to `RED` if no alliance is known at all). `Garden8` is the current concrete
autonomous producing that handoff; new autonomous work can follow its
`Robot`/`PlanBuilder`/`saveEndVals()` pattern directly rather than needing
further abstraction.

## `ConfigOpMode`: pre-start Blackboard editor

`configs/ConfigOpMode.java` extends Pedro's `SelectableOpMode`
(`@TeleOp(name = "Configuration", group = "Configuration")`) and exposes a
folder-based menu of one-shot `BlackboardActionOpMode`s: "Alliance Color"
(writes `AllianceColor.RED`/`BLUE` to `Blackboard.Caption.ALLIANCE`), "Driver"
(writes one of `Driver.ATHARVA`/`SHLOK`/`EASON`/`SIMPLE` to
`Blackboard.Caption.DRIVER`), "Pose" (writes `RED_DEFAULT_START_POSE`,
`BLUE_DEFAULT_START_POSE`, or a zero `Pose()` to `Blackboard.Caption.POSE`),
and "Miscellaneous -> Reset All Blackboard" (`Blackboard::resetAll`). Each
selection runs its action in `init()`, shows a confirmation in `init_loop()`,
and calls `requestOpModeStop()` on `loop()` — so selecting a menu item and
pressing Start both applies the change and exits immediately.

`library/robot/Driver` holds named per-driver preference singletons
(`ATHARVA`, `SHLOK`, `EASON`, `DEFAULT` all `USE_FIELD_CENTRIC = true`;
`SIMPLE` is `false`). `MecanumDrive.setTeleOpDrive(DriveVector)` (the
overload with no explicit `fieldCentric` flag) reads the current
`Blackboard.Caption.DRIVER` value (falling back to `Driver.DEFAULT` if unset)
to decide whether to apply field-centric rotation — so a driver selection made
through `ConfigOpMode` before a match directly changes teleop drive behavior
without any code change.
