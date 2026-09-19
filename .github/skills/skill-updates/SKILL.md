---
name: skill-updates
description: Use this skill to maintain the repository-specific skill guides and companion FTC development documentation.
---

# Skill Documentation Maintenance

When asked to update all skills, update:

1. `.github/skills/README.md`;
2. every relevant `.github/skills/*/SKILL.md`;
3. `ftcdev-20373.html`; and
4. every chapter under `ftcdev-20373/`.

## Required workflow

1. Read `.github/copilot-instructions.md` and this skills index.
2. Find the most recent requested history boundary and inspect both commit
   messages (`git log --oneline`) and changed paths (`git diff --stat`,
   `git show --stat`).
3. Verify every documentation assertion against the current source under
   `TeamCode/src/main/java` and `TeamCode/src/test/java`; commit messages
   summarize intent but do not replace source inspection, and neither does the
   stale, no-longer-authoritative `TeamCode/bin/` compiled-classes output.
4. Search for deleted paths and symbols (`grep`/`glob` across `TeamCode/src`)
   before editing. A rename, deletion, or dependency removal must remove stale
   documentation references everywhere they appear, not just in the file that
   triggered the update.
5. Refresh the narrow guides (single mechanism/package focus) as well as the
   cross-cutting architecture/index pages (`teamcode-library-architecture`,
   `.github/skills/README.md`) so both layers describe the same reality.
6. Review the final diff (`git diff`), re-search for obsolete symbols, and run
   the relevant existing validation (see `build-and-validation`) before
   finishing.

## Source paths to check per topic

When a change touches one of the following areas, re-verify the matching
skill guide(s) against these specific files rather than relying on memory of a
previous pass:

- Mechanisms/lifecycle: `mechanisms/Mechanism.java`, `mechanisms/Robot.java`,
  `mechanisms/RobotDrive.java`, `mechanisms/Intake.java`,
  `mechanisms/Vision.java`, `mechanisms/Shooter.java`,
  `mechanisms/FieldUpdates.java`.
- Vision/localization: `library/hardware/webcam/Limelight.java`,
  `library/IntakeBallPath.java`, `pedroPathing/custom/LimelightPinpointLocalizer.java`.
- Robot profiles/follower: `library/robot/RobotConstants.java`,
  `library/robot/RobotConfigurations.java`,
  `library/robot/drivetrain/MecanumDrive.java`, `Constants.java`.
- Autonomous/planning: `library/planners/PlanBuilder.java`,
  `library/planners/PathPlanner.java`, `library/planners/PathPlannerList.java`,
  `library/planners/paths/*.java`, `library/planners/Common.java`,
  `pedroPathing/Pedro.java`, `pedroPathing/AlliancePoses.java`,
  `library/robot/RobotIvyBuilder.java`.
- Op mode entries/handoff: `tele/TeleOp.java`, `autos/Garden8.java`,
  `configs/Blackboard.java`, `configs/ConfigOpMode.java`.
- Tests/tuning: `tests/ftcSdk/*.java`, `pid/autoTune/*.java`,
  `pedroPathing/Tuning.java`, `src/test/java/.../UnitTestAll.java`.

## Current baseline facts

The current code uses `tele/TeleOp` (with nested `Red_Tele`/`Blue_Tele`) and
`autos/Garden8` (with nested `redalliance`/`bluealliance`) as the concrete
match entries; every profile-managed mechanism extends `Mechanism` and
implements both `periodic(State)` and `stop()`; the aggregate `Robot` owns
mechanism construction, periodic dispatch, and shutdown/handoff via
`saveEndVals()`; autonomous composition goes through `PlanBuilder` and
`PathPlannerList`; `IntakeBallPath` is the vision-driven intake path
generator; `DualGamepad`/`GpBindings` are the two-controller input path; and
the math/control library is plain Java (`library/math/MathUtils`,
`library/control/*`), not Kotlin.

The deleted baseline includes `opModes/` (and its variants pattern),
`IvyAuton`, `IvyTeleOp`, `IvyAuto`, `CommandConverter`, `MathUtils.kt`,
`TestPathPlanner`, `PlannerIntakeVisionAuto`, and the old
`library/hardware/leds/RGBIndicator` location (the directory is now empty; the
class lives under `library/hardware/hardwareDevice/` only).

Never state that `Constants.currentBot` has a permanently or freely selected
profile — it is a compile-time field assignment
(`RobotConstants currentBot = RobotConfigurations.FirstRobot();`) that changes
only when `Constants.java` is edited and rebuilt. Document profile
eligibility, placeholders (e.g. `FirstRobot()`'s zero velocity/heading-PIDF/
centripetal-scaling fields, `FieldUpdates.getGoalPose()` returning `null`), and
scaffolds (e.g. `IntakeBallPath.generateIntakePathFromCam()`'s no-arg overload
returning an empty `PathChain`) as such rather than as finished behavior. Keep
the FTC development hub (`ftcdev-20373.html` and `ftcdev-20373/*.html`)
aligned with the source rather than retaining historical examples purely for
familiarity.

## Cross-checking commit history against source

When a task references "the latest changes" or a specific commit range,
resolve the boundary first (`git log --oneline -20`, `git show --stat
<sha>`), then re-derive facts from the files that changed rather than trusting
the commit subject line alone — subjects like "Add CompositeControllers and
More updates to ShotSolver/Shooter" or "Support dual gamepads and drive
follower modes" describe intent but not the exact resulting API surface
(e.g. whether a params class's `updateFromBot(...)` was actually implemented,
or still returns `null`/`this` unconditionally). Prefer reading the changed
file directly over inferring behavior from the diff summary alone.

## Avoiding accidental formatting errors

Keep Markdown frontmatter exactly two `---` fences around `name:`/
`description:` and avoid stray characters introduced by manual edits (for
example, a leftover `+` from a diff-style edit accidentally left in prose,
which previously appeared before `stop()` in this file's baseline-facts
section). Run `git diff --check` after editing skill files to catch trailing
whitespace or conflict markers before finishing a documentation pass.
