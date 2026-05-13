---
name: build-and-validation
description: Use this skill when asked to build, validate, or sanity-check changes in this FTC Android Studio repository, especially when Gradle, Java, or TeamCode-specific validation steps are involved.
---

# Build and Validation for This FTC Repo

Use this skill when validating changes or diagnosing why the project does or does not build.

## Build layout

- Root Gradle project with modules `:FtcRobotController` and `:TeamCode`
- `TeamCode/build.gradle` applies `build.common.gradle` and `build.dependencies.gradle`
- FTC SDK and Pedro dependencies are declared in `build.dependencies.gradle`

## Known build requirement

- The root `build.gradle` uses Android Gradle Plugin `8.7.0`.
- That plugin requires Java 11 or newer.
- A build launched under Java 8 fails during Gradle configuration before Java sources are compiled.

## Primary validation steps

1. Make sure Gradle runs under Java 11+.
2. Build `:TeamCode:assembleDebug` to catch source and dependency issues in the team module.
3. If needed, build the whole project after TeamCode succeeds.
4. For robot behavior changes, follow the build with driver-station or on-robot validation because many issues are lifecycle or hardware-configuration dependent.

## What static review should focus on

- FTC annotation placement and op mode lifecycle ordering
- Hardware names and directions in `Constants`
- Pedro follower and localizer wiring
- Alliance-specific wrapper classes under `opModes/variants`
- Camera pipeline assumptions versus localization mode

## Current repo-specific warning signs

- Placeholder drivetrain and localization tuning values exist in `Constants`.
- Some behavior will only be trustworthy after real hardware or emulator-equivalent validation.
- This repo does not currently expose a dedicated JVM unit-test suite; most validation is build-time plus robot runtime checks.
