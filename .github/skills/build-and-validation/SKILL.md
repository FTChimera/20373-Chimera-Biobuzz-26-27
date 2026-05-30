---
name: build-and-validation
description: Use this skill when asked to build, validate, or sanity-check changes in this FTC Android Studio repository, especially when Gradle, Java, or TeamCode-specific validation steps are involved.
---

# Build and Validation for This FTC Repo

Use this skill when validating changes or diagnosing why the project does or does not build.

## Build layout

- Root Gradle project with modules `:FtcRobotController` and `:TeamCode`
- `TeamCode/build.gradle` applies `build.common.gradle` and `build.dependencies.gradle`
- `TeamCode/build.gradle` also applies the Sloth load plugin (`dev.frozenmilk.sinister.sloth.load`) and adds the Sloth runtime dependency
- `TeamCode/build.gradle` includes `annotationProcessor` entries for both the FTC op-mode processor and Project Lombok
- FTC SDK and Pedro dependencies are declared in `build.dependencies.gradle`

## Known build requirement

- The root `build.gradle` uses Android Gradle Plugin `8.7.0`.
- That plugin requires Java 11 or newer.
- A build launched under Java 8 fails during Gradle configuration before Java sources are compiled.

## Primary validation steps

1. Make sure Gradle runs under Java 11+.
2. Build `:TeamCode:assembleDebug` to catch source, dependency, annotation-processing, and plugin-wiring issues in the team module.
3. If needed, build the whole project after TeamCode succeeds.
4. For robot behavior changes, follow the build with driver-station or on-robot validation because many issues are lifecycle or hardware-configuration dependent.

## Tooling-specific checks

- If build errors mention missing generated code or annotation processing, confirm the Lombok processor line is still present in `TeamCode/build.gradle` even if the current source set only uses Lombok lightly.
- If build errors mention `dev.frozenmilk`, `Load`, or `Sloth`, check the extra Dairy Foundation repositories, the `buildscript` classpath entry, the applied plugin line, and the Sloth runtime dependency in `TeamCode/build.gradle`.
- Treat Sloth and Lombok as intentional build wiring, not dead code to remove casually during cleanup.

## What static review should focus on

- FTC annotation placement and op mode lifecycle ordering
- Central subsystem factories and params instances in `Constants`
- Mechanism constructors accepting the expected params object
- Pedro follower and localizer wiring
- Alliance-specific wrapper classes under `opModes/variants`
- Camera pipeline assumptions versus localization mode

## Centralized mechanism validation checklist

When a change adds or modifies a mechanism, confirm all of these:

1. The mechanism or wrapper class defines its own params type.
2. `Constants` owns the shared params instance used for Dashboard/panels.
3. `Constants` exposes a `create...(...)` factory for the mechanism.
4. Op modes use the `Constants` factory instead of direct construction.
5. Drive-related configuration still lives in `RobotConstants` / `RobotConfigurations`, not mixed into unrelated mechanism params.

## Current repo-specific warning signs

- Placeholder drivetrain and localization tuning values still exist in `RobotConfigurations` and related config objects.
- Some behavior will only be trustworthy after real hardware or emulator-equivalent validation.
- This repo does not currently expose a dedicated JVM unit-test suite; most validation is build-time plus robot runtime checks.
- Documentation or code that still references a `Mechanisms.java` factory surface is stale; the repo now uses `Constants` factories.
- Build guidance that ignores Lombok or Sloth is stale; both are now part of the TeamCode module wiring.
