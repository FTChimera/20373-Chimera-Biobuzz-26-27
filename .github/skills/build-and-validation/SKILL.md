---
name: build-and-validation
description: Use this skill when asked to build, validate, or sanity-check changes in this FTC Android Studio repository, especially when Gradle, Java, or TeamCode-specific validation steps are involved.
---

# Build and Validation for This FTC Repo

Use this skill when validating changes or diagnosing why the project does or does not build.

## Build layout

- Root Gradle project with modules `:FtcRobotController` and `:TeamCode`
- The root `build.gradle` currently sets `kotlin_version = '2.1.20'`
- `TeamCode/build.gradle` applies `build.common.gradle` and `build.dependencies.gradle`
- `TeamCode/build.gradle` applies `org.jetbrains.kotlin.android`, so `:TeamCode:compileDebugKotlin` is part of a normal TeamCode build
- `TeamCode/build.gradle` also applies the Sloth load plugin (`dev.frozenmilk.sinister.sloth.load`) and adds the Sloth runtime dependency
- `TeamCode/build.gradle` includes `annotationProcessor` entries for both the FTC op-mode processor and Project Lombok
- FTC SDK and Pedro dependencies are declared in `build.dependencies.gradle`
- TeamCode now contains both Java and Kotlin sources, including validation or learning op modes

## Known build requirement

- The root `build.gradle` uses Android Gradle Plugin `8.7.0`.
- That plugin requires Java 11 or newer.
- A build launched under Java 8 fails during Gradle configuration before Java sources are compiled.

## Primary validation steps

1. Make sure Gradle runs under Java 11+.
2. Build `:TeamCode:assembleDebug` to catch source, dependency, annotation-processing, and plugin-wiring issues in the team module.
3. Run `:TeamCode:lint` when you want Android/lifecycle/static warnings as well.
4. If needed, build the whole project after TeamCode succeeds.
5. `:TeamCode:testDebugUnitTest` currently exists but usually reports `NO-SOURCE`, so treat build + lint + robot/runtime validation as the primary checks.
6. For robot behavior changes, follow the build with driver-station or on-robot validation because many issues are lifecycle or hardware-configuration dependent.

## Tooling-specific checks

- If build errors mention missing generated code or annotation processing, confirm the Lombok processor line is still present in `TeamCode/build.gradle` even if the current source set only uses Lombok lightly.
- If build errors mention Kotlin plugin setup, check the root `kotlin_version`, the root Kotlin classpath entry, and the `org.jetbrains.kotlin.android` plugin in `TeamCode/build.gradle`.
- If build errors mention `dev.frozenmilk`, `Load`, or `Sloth`, check the extra Dairy Foundation repositories, the `buildscript` classpath entry, the applied plugin line, and the Sloth runtime dependency in `TeamCode/build.gradle`.
- Treat Sloth and Lombok as intentional build wiring, not dead code to remove casually during cleanup.

## What static review should focus on

- FTC annotation placement and op mode lifecycle ordering
- `Constants` convenience factories versus the underlying `RobotConstants.createMechanism(...)` flow
- `RobotConstants.Mechanisms` registration as a `Set`, including `configureDriveMechanisms()` for the drive stack
- Mechanism constructors matching the expected `(HardwareMap, RobotConstants)` signature
- `RobotConfigurations` registration through `.addMechanism(...)`
- Vision or LED wrappers that should skip hardware lookup when the mechanism is disabled
- Pedro follower and localizer wiring
- `EquationCurve` / `Polynomial` contracts if custom path geometry changed
- Alliance-specific wrapper classes under `opModes/variants`
- Camera pipeline assumptions versus localization mode
- Test/proof-of-concept code under `tests/` versus shared match logic under `opModes/`

## Centralized mechanism validation checklist

When a change adds or modifies a mechanism, confirm all of these:

1. The mechanism or wrapper extends `Mechanism` if it should participate in the normal robot-subsystem flow.
2. Its constructor matches `(HardwareMap, RobotConstants)`.
3. The active robot profile registers it with `.addMechanism(...)`.
4. If it is part of the drive stack, check whether `configureDriveMechanisms()` should own the default registration.
5. `Constants` exposes or reuses a `create...(...)` helper when callers should not spell out `currentBot.createMechanism(...)` directly.
6. If the class can be disabled by profile selection, make sure it decides that before touching hardware.
7. Op modes use the normal factory path instead of direct construction.
8. Drive-related configuration still lives in `RobotConstants` / `RobotConfigurations`, not mixed into unrelated mechanism logic.

## Current repo-specific warning signs

- Placeholder drivetrain and localization tuning values still exist in `RobotConfigurations` and related config objects.
- Some behavior will only be trustworthy after real hardware or emulator-equivalent validation.
- This repo does not currently expose a dedicated JVM unit-test suite; most validation is build-time plus robot runtime checks.
- Documentation or code that still assumes every subsystem is wired by a central params singleton in `Constants` is stale; uniform construction now centers on `Mechanism` plus `RobotConstants.createMechanism(...)`.
- Build guidance that ignores Lombok, Sloth, or Kotlin plugin wiring is stale.
- `tests/`, `tests/ftcSdk`, and `learning/` intentionally contain validation or teaching code; build review should distinguish those from shared production architecture.
