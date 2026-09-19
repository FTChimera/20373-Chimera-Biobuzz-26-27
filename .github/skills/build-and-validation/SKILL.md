---
name: build-and-validation
description: Use this skill to build and validate this FTC Android Studio repository.
---

# Build and Validation

Use the repository Gradle wrapper from the repository root.

## Baseline first

1. Inspect `git status --short` before changing anything.
2. Do not overwrite unrelated work already present in the tree.
3. Read the edited source and its direct tests before selecting validation.
4. Inspect `Constants`, `RobotConstants`, and `RobotConfigurations` for profile effects.
5. Inspect `tele/TeleOp.java` and `autos/Garden8.java` for match-entry behavior.
6. Inspect `mechanisms/Robot.java` and `Mechanism.java` for lifecycle changes.
7. Inspect `library/planners/` and `IntakeBallPath.java` for autonomous changes.
8. Inspect `tests/`, `tests/ftcSdk/`, `pid/autoTune/`, and `src/test/java` for existing coverage.

## Fast source checks

- Use `git diff --check` after every documentation or source edit.
- Use `git diff --name-only -- <edited paths>` to confirm the intended scope.
- Search exact renamed or deleted symbols in the files you changed.
- Search Java source before claiming an API is present or absent.
- Prefer `rg` when available; otherwise use `grep -R`.
- Keep search output focused with file globs and explicit paths.

## JVM unit tests

- The unit-test suite entry is `TeamCode/src/test/java/.../UnitTestAll.java`.
- Run the existing TeamCode JVM test task when Java library code changes:
  `./gradlew :TeamCode:test`.
- If that task is unavailable in the checked-out SDK, report the actual Gradle failure.
- Do not invent a replacement test command.
- Run a focused existing test class only when Gradle already supports that selection.
- Re-run the relevant test after fixes rather than reporting the first failure alone.

## Android compilation

- For TeamCode source, run:
  `./gradlew :TeamCode:assembleDebug`.
- This catches Java compilation, Android resource processing, and dependency resolution.
- The first Android build can take longer while Gradle resolves caches.
- Do not run a daemon or watcher as validation.
- Do not claim hardware deployment validates code that did not compile.

## FTC-specific checks

- Hardware-dependent op modes cannot be fully exercised by a desktop JVM test.
- Compile a changed FTC entry point before deploying it to the Control Hub.
- Confirm annotations, names, and preselected teleop values in the source.
- Confirm device names and profile registration against the active robot configuration.
- Verify a mechanism is configured before using `Robot.require(...)`.
- Verify mode consistency: a `State` passed to `Robot.periodic(State)` must match the robot mode.
- Verify autonomous stop paths call the aggregate robot shutdown path.

## Lifecycle validation

- `Mechanism` subclasses must implement `periodic(State)` and `stop()`.
- `Robot.periodic(State)` sets state context and dispatches all registered mechanisms.
- `Robot.start()` starts its helper and the AprilTag vision pipeline when present.
- `Robot.stop()` stops mechanisms, closes the helper, and performs its normal handoff.
- Test autonomous handoff only after at least one periodic update.
- Test teleop initialization with both a saved pose and the default alliance pose.

## Planner and vision validation

- Build planner sequences through `PlanBuilder` and inspect legal position transitions.
- Verify `Garden8` schedules its built plan and executes the Ivy scheduler in `loop()`.
- Treat loading-side planner modules as incomplete unless source proves otherwise.
- For `IntakeBallPath`, test null/empty detections and invalid requested ball counts.
- Verify detector filtering, coordinate conversion, and safe failure behavior.
- Remember the no-argument intake-path method is an empty-chain scaffold.

## Gamepad and drive validation

- Call `DualGamepad.update()` once per teleop loop before reading bindings.
- Confirm `State` receives `DualGamepad.result()` when commands use bindings.
- Validate both controller paths with the existing gamepad tests.
- Check drive follower and teleop drive transitions on the configured profile.
- Do not infer field-centric behavior from joystick input alone; inspect `RobotDrive`.

## Documentation-only changes

- Run `git diff --check`.
- Search the changed guides for obsolete symbols or paths.
- Ensure each edited skill guide is between 100 and 300 lines when requested.
- Review the rendered Markdown structure and frontmatter fences.
- Do not modify unowned skill files merely because a global index is stale.

## Final report

- State the exact commands run.
- State whether each command passed, failed, or was unavailable.
- Include concise failure output when validation cannot run.
- Report the changed file list.
- Never represent an unrun build, test, or robot deployment as successful.

## Command conventions

- Use `--console=plain` when a saved CI-style log is useful.
- Preserve the first actionable compiler error when a build fails.
- Separate environmental dependency failures from source regressions.
- Do not run `clean` by default; it removes useful incremental build evidence.
- Use Control Hub deployment only after the relevant build succeeds.
- Keep validation commands scoped to the changed module when possible.
