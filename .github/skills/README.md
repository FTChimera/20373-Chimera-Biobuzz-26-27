# Project Skills Index

This folder contains repo-specific skills for working on `20373-Chimera-Biobuzz-26-27`.

## Existing broad skills

- `build-and-validation/`
- `ftc-sdk-opmodes/`
- `localization-and-vision/`
- `pedro-pathing-and-ivy/`
- `teamcode-library-architecture/`

## Additional focused skills

- `robot-configurations-and-follower-builder/` — robot profiles, `RobotConstants`, `RobotConfigurations`, `addMechanism(...)`, and Pedro follower / mechanism creation
- `mechanisms-and-hardware-wrappers/` — wrapper, custom `HardwareDevice`, and subsystem patterns in `library/hardware` and `mechanisms`
- `drivetrain-and-drivevector/` — drive math, `DriveVector`, `Drivetrain`, and `MecanumDrive`
- `opmode-variants-and-pose-handoff/` — shared op mode bases, variants, and `configs.Blackboard` pose / driver-state handoff
- `ivy-command-patterns/` — Pedro Ivy scheduling, command construction, and `CommandConverter`
- `ftc-sdk-hardwaremap-and-lifecycle/` — FTC SDK API usage, `HardwareMap`, annotations, and lifecycle rules
- `limelight-wrapper-and-pipelines/` — Limelight wrapper usage and pipeline conventions
- `autonomous-path-patterns/` — state-machine, `PathChain`, and custom-curve patterns used in autonomous
- `gamepad-and-driver-input/` — raw FTC gamepad APIs plus `GamepadEx` / `DualGamepad` wrapper conventions
- `tests-and-tuning-opmodes/` — test op modes, Pedro tuning, and validation habits
- `skill-updates/` — how to update `.github/skills/README.md` and the relevant `SKILL.md` files from recent commit history without over-editing unrelated skills
- `build-and-validation/` — Has an additional section on validating all of the files/classes in the `TeamCode` folder.

## Notes

Most skills are intentionally narrow and repo-specific. Prefer the focused skill when the task clearly matches one of these areas, and use the broader skills for cross-cutting changes.

Across the skill set, the current house style is:

- put drive/profile data in `library/robot/RobotConstants` and `RobotConfigurations`
- register optional subsystems per robot profile with `RobotConstants.configureDriveMechanisms()` and `RobotConstants.addMechanism(...)`
- have reusable subsystems and wrappers extend `mechanisms/Mechanism`
- give profile-managed subsystems a constructor shaped like `(HardwareMap, RobotConstants)` so `RobotConstants.createMechanism(...)` can instantiate them uniformly
- use `Constants.create...(...)` helpers as the normal op-mode-facing convenience entry points when a subsystem is used broadly

Current repo details worth remembering while using these skills:

- drive motor directions are represented with `library/hardware/actuators/Direction.java`, then converted to FTC SDK directions at the wrapper/profile boundary
- IMU hardware names and REV Hub logo/USB orientation belong in `RobotConstants` / `RobotConfigurations`; create and initialize the active-profile IMU through `Constants.createIMU(hardwareMap)`, then reset yaw only at the caller's chosen time
- custom SDK-registerable hardware now lives under `library/hardware/hardwareDevice/`, including `RevPotentiometer`, `SlidesMotor`, and the device-level `RGBIndicator`
- `library/OpModeHelper.kt` is available as a lightweight manual-bulk-read and loop-`dt` helper for op modes that need one
- `TeamCode/build.gradle` wires Project Lombok annotation processing and the Sloth load / hot-reload tooling, so build troubleshooting should account for both
- `build.dependencies.gradle` keeps JitPack plus `com.github.aaravdhawan25.FtcAutoTune:pidautotuner-core:v0.3.7`; the repo-local `pid/autoTune/master/PIDMaster` and `DualMotorPIDMaster` classes now provide the FTC hardware adapters around that core
- `mechanisms/Slides1.java` is the current profile-style slide mechanism shell layered on top of `library/hardware/hardwareDevice/SlidesMotor.java`
- `tests/MecanumTeleOp.java` is a first-class iterative test op mode for robot-centric drive, field-centric drive, IMU yaw reset, and manual motor-direction checks
- `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` now set their stick curve once on `MecanumDrive` via `setStickCurve(...)` and then pass raw `DriveVector` values, so future drive edits should avoid double-applying stick shaping in the op mode layer
- `tests/MecanumTeleOp.java`, `tests/PreseasonWebcamAuto.java`, `tests/ftcSdk/RGBIndicatorTest.java`, and `tests/ftcSdk/SlidesMotorTest.java` now intentionally pull hardware names from `RobotConfigurations` or the owning `*_PARAMS` class instead of stale test-only string literals
- `tests/ftcSdk/androidTest.java` is now a scripted SDK-test base that runs one validation flow, sleeps briefly, and then calls `requestOpModeStop()` instead of spinning in an empty active loop
- `library/math/Equation.java`, `Term.java`, and `Polynomial.java` now form a provisional equation-math layer; `Polynomial` is the implemented reusable piece today, while `Term` is still scaffold-only
- `pedroPathing/custom/EquationCurve.java` now accepts a generic `Equation` plus a `HeadingInterpolator`, so custom path geometry is no longer limited to raw `Polynomial` inputs
- `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` are Java and Kotlin scratch iterative Ivy autonomous examples that build `PathChain`s, schedule `PedroCommands.follow(...)`, and run `Scheduler.execute()` inside `loop()`
- `learning/ExampleIvyWithSubsystems.java` is a Java scratch scaffold that compiles with its unfinished `autoAlign()` step commented out; it still uses a direct intake `DcMotor` for teaching and is not the profile-managed match architecture
- `java-autogen/Ftc-PP-toJavaAuto-Ivy.html`, `java-autogen/Ftc-PedroPathing-PathBuilder-toJavaAuto.html`, and `java-autogen/Ftc-PP-toJavaAuto-Ivy.js` are browser-side generators for turning Pedro/Turtle Tracer path data or builder-authored path drafts into Java autonomous skeletons, optionally using the Ivy command-base style
