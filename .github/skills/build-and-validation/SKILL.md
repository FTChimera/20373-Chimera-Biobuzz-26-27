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
- `TeamCode/build.gradle` declares `testImplementation` and `androidTestImplementation`, but the current repo has no `TeamCode/src/test/` or `TeamCode/src/androidTest/` tree; validation op modes live under `src/main/java`
- `java-autogen/` contains browser-side HTML/JS generators and is outside the Gradle-compiled Android source set
- Custom SDK-registerable hardware now also lives under `library/hardware/hardwareDevice/`, including `RevPotentiometer`, `SlidesMotor`, and a device-level `RGBIndicator`
- `build.dependencies.gradle` now pulls JitPack plus `com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.3.7`; repo-local `pid/autoTune/master/PIDMaster.java` and `DualMotorPIDMaster.java` provide the FTC-facing tuning layer
- TeamCode now includes `pid/autoTune/` relay-feedback tuner op modes, the repo-local master adapters, and `pid/autoTune/TuningConstants.java`

## Known build requirement

- The root `build.gradle` uses Android Gradle Plugin `8.7.0`.
- Treat Java 11 or newer as the intended Gradle runtime for this project.
- The checked-in `gradle.properties` currently caps Gradle at `org.gradle.jvmargs=-Xmx1024M`; if a local build fails for memory reasons, separate that from source errors before treating it as a code regression.
- `learning/ExampleIvyWithSubsystems.java` compiles with its unfinished `autoAlign()` step commented out. Treat it as a teaching scaffold, not as a profile-managed subsystem reference.

## Primary validation steps

1. Make sure Gradle runs under Java 11+.
2. Build `:TeamCode:assembleDebug` to catch source, dependency, annotation-processing, and plugin-wiring issues in the team module.
3. Run `:TeamCode:lint` when you want Android/lifecycle/static warnings as well.
4. If needed, build the whole project after TeamCode succeeds.
5. `:TeamCode:testDebugUnitTest` currently exists but usually reports `NO-SOURCE`, so treat build + lint + robot/runtime validation as the primary checks.
6. For robot behavior changes, follow the build with driver-station or on-robot validation because many issues are lifecycle or hardware-configuration dependent.
7. If a change only touches `java-autogen/`, review the generated HTML/JS directly; Gradle will not validate that tooling until its output is copied into `TeamCode/`.

## When the user asks for a full `TeamCode` audit

This repo sometimes gets requests like: “Analyze the entire TeamCode codebase for errors or potential pitfalls. Ask me for any questions and clarifications.”

Treat that as a repo-wide audit request, not as permission to start fixing code immediately.

### Default contract for this request

- If any audit-shaping items are still ambiguous, state the assumptions you used, such as:
  - which areas are highest priority
  - whether zero/default constants are intentional preseason placeholders or bugs to fix now
  - whether the user wants report-only output or also wants follow-up fixes
  - whether there are any explicit exceptions that are allowed even in a report-only audit
- The default outcome is a report: findings, risks, validation results, and recommended follow-up work.
- Default to listing every issue and every verification step; do **not** change files unless the user explicitly asks for fixes.
- If the user provides an exception, edit files **strictly** for that exception and keep the rest of the audit report-only.
- If any files are changed under an allowed exception, list every changed file in the final report and rerun focused validation on those files plus the relevant build/lint checks.

### Recommended audit workflow

1. Read the repo instructions and the relevant skills first.
2. Map the `TeamCode/` package structure so you know which areas are op modes, mechanisms, library code, configs, Pedro helpers, `pid/autoTune` tuners, tests, and learning/scratch code.
3. Inspect the current `Constants`, `RobotConstants`, `RobotConfigurations`, `opModes/`, `tests/`, `pid/autoTune/`, and `learning/` sources before drawing conclusions about “stale” code or architecture intent.
4. Run the hard validation passes.

```zsh
./gradlew :TeamCode:assembleDebug --console=plain
./gradlew clean :TeamCode:assembleDebug --console=plain
./gradlew :TeamCode:lint --console=plain
./gradlew :TeamCode:testDebugUnitTest --console=plain
```

5. If lint succeeds, inspect `TeamCode/build/reports/lint-results-debug.txt` or the HTML report, not just the Gradle success banner.
6. If the audit allows an exception-based edit, run file-scoped diagnostics on the edited files afterward in addition to the module build.
7. Follow the automated checks with a full static review, because many FTC issues are architectural, lifecycle-related, or hardware-config dependent rather than pure compiler errors.

### High-value static review checklist

Use as many of these checks as are relevant:

#### 1. FTC lifecycle and annotation correctness

- `@TeleOp` / `@Autonomous` only on concrete selectable classes
- no `waitForStart()` inside iterative `OpMode` classes
- one-time setup in `init()` or before `waitForStart()`, not every loop
- important persistence or cleanup in `stop()` where the repo already relies on that pattern
- no accidental busy loops or empty active loops that waste CPU

#### 2. Mechanism/factory architecture

- op modes should prefer `Constants.create...(...)`
- profile-managed subsystems should extend `Mechanism`
- constructors should match `(HardwareMap, RobotConstants)`
- optional mechanisms should check enablement before `hardwareMap.get(...)`
- `RobotConfigurations` should register profile-managed mechanisms with `.addMechanism(...)` or `configureDriveMechanisms()`
- direct `new Intake(...)`, `new Drivetrain(...)`, `new Limelight(...)`, etc. should be treated as suspicious unless intentionally internal

Useful search:

```zsh
git --no-pager grep -nE 'new (Intake|Drivetrain|MecanumDrive|Limelight|LogitechWebcam)\(' -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

#### 3. Hardware-name and profile drift

- drivetrain motor names should come from `RobotConstants` / `RobotConfigurations`
- IMU hardware name and REV Hub logo/USB orientation should come from `RobotConstants` / `RobotConfigurations` through `Constants.createIMU(...)`
- mechanism-specific names should come from the owning nested `*_PARAMS` class when they are not yet profile-managed
- tests should not reintroduce stale standalone hardware strings when centralized names already exist
- drive directions should stay aligned between `RobotConfigurations`, `Drivetrain`, and follower construction

Useful search:

```zsh
git --no-pager grep -nE 'hardwareMap\.get\(.*".+"' -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

#### 4. Custom FTC `HardwareDevice` versus profile-managed wrapper confusion

- `library/hardware/hardwareDevice/` classes like `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator` are direct FTC devices
- `library/hardware/leds/RGBIndicator` is a separate profile-managed wrapper/mechanism layer
- do not confuse direct `hardwareMap.get(CustomType.class, ...)` code with the `Mechanism` factory path

#### 5. Drive and Pedro behavior

- only one consistent `follower.update()` loop per active control path
- `DriveVector` sign/curve behavior should stay consistent with teleop expectations
- field-centric math and heading-reset logic should agree between `DriveVector`, `Drivetrain`, `MecanumDrive`, and test op modes
- `RobotConstants.createFollower(...)` should stay aligned with the active localizer and drivetrain config

#### 6. Vision and localization assumptions

- Limelight pipeline choice should be explicit
- raw Limelight proof-of-concept code under `tests/` should be recognized as test code, not the shared production wrapper pattern
- placeholder Limelight geometry or follower tuning should only be reported as “intentional placeholders” if the user confirmed that intent
- `LimelightPinpointLocalizer` is deprecated and not the default follower path

#### 7. Blackboard and pose handoff

- `Blackboard` reads should be checked for null/default semantics
- pose handoff logic should distinguish between “default seed value” and “meaningful restored pose”
- driver preference flow should stay centralized through `Blackboard.Caption.DRIVER`

#### 8. Test op mode safety and purpose clarity

- test op modes should use safe power levels when motors may move unexpectedly
- each test should have one clear purpose
- proof-of-concept code should be documented as such
- test op modes should stay aligned with centralized hardware naming and current wrappers/factories where appropriate
- `pid/autoTune/` op modes are calibration utilities, not match logic; direct motor-name configuration through `TuningConstants` is expected because repo-local master adapters own FTC hardware access while `pidautotuner-core` supplies the tuning primitives

#### 9. Control/utilities robustness

- broad `catch (Exception ...)` blocks may hide real failures
- `printStackTrace()` in library code is usually a warning sign, even if acceptable temporarily in tests
- helper utilities like `OpModeHelper.kt`, PID controllers, timers, and math helpers should be checked for state-machine or dimension assumptions

#### 10. Build/tooling warning signs

- non-incremental annotation processors
- lint warnings in generated reports
- unused resources that may indicate stale configuration
- TODO/FIXME markers, placeholder comments, or deprecated experiment code that no longer matches active architecture

Useful search:

```zsh
git --no-pager grep -nE 'TODO|FIXME|printStackTrace|catch \(Exception|while \(opModeIsActive\(\)\);' -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

### How to present the audit results

- Separate verified build/lint failures from reasoning-based risks.
- Group findings by severity or priority area.
- Call out intentional preseason placeholders separately from probable bugs when the task context clarifies that distinction.
- If you changed anything under an exception, list every file changed and explain that the rest of the audit remained report-only.
- End with recommended next-fix targets rather than silently applying broad cleanup.

## Tooling-specific checks

- If build errors mention missing generated code or annotation processing, confirm the Lombok processor line is still present in `TeamCode/build.gradle` even if the current source set only uses Lombok lightly.
- If build errors mention Kotlin plugin setup, check the root `kotlin_version`, the root Kotlin classpath entry, and the `org.jetbrains.kotlin.android` plugin in `TeamCode/build.gradle`.
- If build errors mention `dev.frozenmilk`, `Load`, or `Sloth`, check the extra Dairy Foundation repositories, the `buildscript` classpath entry, the applied plugin line, and the Sloth runtime dependency in `TeamCode/build.gradle`.
- If build errors mention `com.aaravdhawan25`, `pidautotuner`, or missing tuner classes, check the JitPack repository entry, the `pidautotuner-core` dependency line, and the repo-local `pid/autoTune/master/` adapter classes.
- Treat Sloth and Lombok as intentional build wiring, not dead code to remove casually during cleanup.

## What static review should focus on

- FTC annotation placement and op mode lifecycle ordering
- `Constants` convenience factories versus the underlying `RobotConstants.createMechanism(...)` flow
- `RobotConstants.Mechanisms` registration as a `Set`, including `configureDriveMechanisms()` for the drive stack
- Mechanism constructors matching the expected `(HardwareMap, RobotConstants)` signature
- `RobotConfigurations` registration through `.addMechanism(...)`
- Vision or LED wrappers that should skip hardware lookup when the mechanism is disabled
- Distinguishing profile-managed wrappers like `library.hardware.leds.RGBIndicator` from custom SDK hardware devices like `library.hardware.hardwareDevice.RGBIndicator`
- Pedro follower and localizer wiring
- `OpModeHelper.kt` lifecycle state handling and manual Lynx bulk-caching usage when that utility is touched
- `EquationCurve` / `Polynomial` contracts if custom path geometry changed
- Alliance-specific wrapper classes under `opModes/variants`
- Camera pipeline assumptions versus localization mode
- Test/proof-of-concept code under `tests/` versus shared match logic under `opModes/`
- The `pid/autoTune/` package as a tuning-only exception to the normal `Constants.create...(...)` subsystem factory flow

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
9. If the code intentionally bypasses the wrapper path in a test, check that hardware names still come from `RobotConfigurations` or the owning `*_PARAMS` class instead of stale literals.

## Current repo-specific warning signs

- Placeholder drivetrain and localization tuning values still exist in `RobotConfigurations` and related config objects.
- `Constants.currentBot` currently points at `RobotConfigurations.OldDECODERobotTesting()`, which registers the drive stack plus `Intake`, `Limelight`, and wrapper `RGBIndicator`. `Constants.createWebcam(...)` and `Constants.createSlides1(...)` are still runtime assertion points unless the active profile is changed or expanded.
- Several tests now intentionally centralize hardware names through `Constants.currentBot` or nested params classes; new test edits should preserve that instead of reintroducing ad hoc strings.
- `pid/autoTune/` is intentionally driver-station-visible tuning code that uses raw motor names and direction booleans from `TuningConstants` rather than the profile-managed mechanism factory path.
- Some behavior will only be trustworthy after real hardware or emulator-equivalent validation.
- This repo does not currently expose a dedicated JVM unit-test suite; most validation is build-time plus robot runtime checks.
- Documentation or code that still assumes every subsystem is wired by a central params singleton in `Constants` is stale; uniform construction now centers on `Mechanism` plus `RobotConstants.createMechanism(...)`.
- Build guidance that ignores Lombok, Sloth, or Kotlin plugin wiring is stale.
- `tests/`, `tests/ftcSdk`, and `learning/` intentionally contain validation or teaching code; build review should distinguish those from shared production architecture.
