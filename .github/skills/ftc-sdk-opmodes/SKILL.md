---
name: ftc-sdk-opmodes
description: Use this skill when asked to create or edit FTC SDK op modes, wire hardware through HardwareMap, add TeleOp or Autonomous classes, or debug FTC lifecycle issues in this TeamCode project.
---

# FTC SDK Op Modes for This Repository

Use this skill when working on FTC SDK code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

## Upstream `~/.gradle` lookup

If an op-mode behavior depends on FTC SDK or Pedro infrastructure rather than
repo-local code, inspect `~/.gradle/caches/modules-2/files-2.1/` using the
versions pinned in `build.dependencies.gradle`.

- `org.firstinspires.ftc/RobotCore`, `org.firstinspires.ftc/Hardware`, and
  `org.firstinspires.ftc/Vision` for SDK-side op-mode behavior
- `com.pedropathing/ivy` for Ivy command-driven op modes
- `com.pedropathing/telemetry` for `pedroPathing/Tuning.java` op-mode selection
  or telemetry registration issues

## Repository context

- The app uses FTC SDK `11.2.1`.
- `TeamCode/` is the team module; `FtcRobotController/` is the controller app plus SDK samples.
- Shared robot constants and mechanism factories live in `Constants.java`, with drive-profile and follower configuration under `library/robot/`.
- The `OldDECODERobotTesting` and `FirstRobot` profiles exist. `Constants.currentBot` selects the profile used for profile-managed factories.
  `configureDriveMechanisms()` supplies the normal `RobotDrive` registration,
  while each profile explicitly registers any optional mechanism it supports.
- Custom FTC `HardwareDevice` classes now live under `library/hardware/hardwareDevice/` for direct SDK registration and `hardwareMap.get(...)` usage.
- Pose and driver-preference handoff between op modes lives in `configs/Blackboard.java`.
- Shared bases and concrete match op modes live under `opModes/`; `variants/` currently holds thin wrappers such as `Blue_TeleOp`, `Red_TeleOp`, and `Test_IvyAuton`, while `IvyTeleOp` is an annotated class in the `opModes/` root.
- Validation and proof-of-concept op modes live under `tests/`.
- FTC AutoTune calibration op modes live under `pid/autoTune/`.
- One-shot configuration op modes live under `configs/`.
- Learning or scratch op modes live under `learning/`.
- Browser-side autonomous generators live under `java-autogen/`; they are tooling inputs to op-mode authoring, not selectable FTC classes by themselves.

## Use these patterns

1. Put reusable op mode logic in a shared base class under `opModes/`.
2. When you use the thin-wrapper pattern, put the driver-station-visible wrappers in `opModes/variants/` and annotate those with `@TeleOp` or `@Autonomous`.
3. Resolve subsystem construction through `Constants.create...(...)` instead of direct constructors in op modes.
4. Let `Constants.create...(...)` delegate to the active robot profile when the subsystem is profile-managed.
5. If teleop needs the last autonomous pose or configured driver preference, read and write `configs.Blackboard` intentionally.
6. Use `tests/` for proof-of-concept or validation flows that are not part of the shared match architecture.
7. Use `pid/autoTune/` for driver-station-visible relay-feedback tuning utilities that use repo-local FTC master adapters backed by `pidautotuner-core`.
8. If an op mode wants loop timing or manual bulk-cache clearing, reuse `library/OpModeHelper.kt` instead of inventing a second helper with the same role.

## Central mechanism-creation convention

When an op mode needs a subsystem:

- prefer `Constants.createRobot(hardwareMap, RobotMode...)` in shared op
  modes, create a matching `State`, then use `robot.requireIntake()` or
  `robot.get(...)` for profile-managed subsystems
- call `Constants.createVision(hardwareMap)` or retrieve `Vision` from
  `Robot`; `Limelight` is the controller owned privately by `Vision`
- call `Constants.createRobot(hardwareMap, RobotMode...)` when a shared op
  mode wants the whole registered mechanism set, then use `robot.requireDrive()`
  or optional `robot.get(...)` lookups as needed
- call `Constants.createDrive(hardwareMap)` when you intentionally want just the public drive mechanism
- call `Constants.createIMU(hardwareMap)` instead of directly retrieving and orienting an `IMU`
- call `Constants.createRGB(hardwareMap)` or `Constants.createWebcam(hardwareMap)` when those profile-managed wrappers are needed

Every profile-managed `Constants.create...(...)` helper first calls `Constants.assertHasMechanism(...)`, so the profile selected by `Constants.currentBot` must register the requested mechanism class before the helper will run successfully. `createFollower(...)` and `createIMU(...)` are profile-configured helpers, not mechanism factories.

When adding a brand-new mechanism, add all three pieces together:

1. make it extend `Mechanism` if it should participate in the standard robot-subsystem flow
2. give it the `(HardwareMap, RobotConstants)` constructor expected by `RobotConstants.createMechanism(...)`
3. register it in `RobotConfigurations` with `.addMechanism(...)`
4. add a `Constants.create...(...)` helper if op modes should access it directly

The important repo contract is that profile-managed subsystems can be created uniformly from the active `RobotConstants` instance.

## Current repo conventions

- Teleop drive is usually routed through `RobotDrive`, which wraps `MecanumDrive`.
- Intake is wrapped by `mechanisms/Intake.java`, and `Constants.createIntake(...)` is the intended factory path when the active profile registers `Intake.class`.
- Limelight usage is profile-managed by `mechanisms/Vision.java`, and
  `Constants.createVision(...)` is the intended factory path when the active
  profile registers `Vision.class`.
- Logitech webcam has a `Constants` helper, but `Constants.currentBot` does not currently register that mechanism.
- `library/hardware/hardwareDevice/SlidesMotor.java` is the current direct SDK-level slides device surface.
- Ivy-based op modes use `Scheduler.reset()`, `Scheduler.schedule(...)`, and `Scheduler.execute()`.
- `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` are unannotated Java and Kotlin scratch iterative Ivy autonomous examples, not driver-station-visible match op modes.
- `learning/ExampleIvyWithSubsystems.java` is an unannotated scratch scaffold. Its unfinished `autoAlign()` step is commented out, and it initializes a direct intake `DcMotor` from `Intake.INTAKE_PARAMS`; do not use it as the profile-managed subsystem reference.
- Non-Ivy op modes call subsystem update methods directly inside `loop()` or the main `while (opModeIsActive())` block.
- `opModes/TeleOp.java` and `opModes/IvyTeleOp.java` both construct
  `RobotMode.TELEOP` aggregates and matching `State` objects, then start the
  robot after `waitForStart()`. `TeleOp` requires `RobotDrive` and `Vision`
  while leaving `Intake` optional; `IvyTeleOp` treats both `Intake` and
  `Vision` as optional.
- `tests/MecanumTeleOp.java` is a current iterative FTC test op mode for raw mecanum drive, IMU yaw reset, and manual direction checks.
- `tests/MecanumTeleOp.java` now sources drivetrain motor names and directions from `Constants.currentBot` instead of separate test-only strings.
- `tests/MecanumTeleOp.java` and `tests/FieldCentricMecanumTeleOp.java` source IMU lookup and hub orientation from `Constants.createIMU(hardwareMap)`; yaw reset remains an explicit driver-control action.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept `LinearOpMode` that
  uses `Vision` for color-based artifact tracking.
- `tests/MotorDirectionTest.java` is now a looping low-speed sanity check that keeps telemetry live while you flip drivetrain or intake directions.
- `pid/autoTune/PositionPIDTunerOpMode.java`, `VelocityPIDFTunerOpMode.java`, and `DualVelocityPIDFTunerOpMode.java` are `@TeleOp` calibration utilities in group `FtcAutoTune`; they use repo-local `PIDMaster` / `DualMotorPIDMaster` adapters plus `TuningConstants`, with `pidautotuner-core` providing the tuning primitives.
- `configs/ConfigOpMode.java` is the single iterative configuration menu for blackboard alliance, driver, pose, and reset mutations; it no longer delegates those actions to separate annotated config op modes.

## Rules

- Keep FTC annotations on concrete, selectable classes only.
- Prefer adding alliance-specific wrappers over duplicating whole op modes.
- Do not bypass mechanism wrappers with raw `DcMotorEx` or `Limelight3A` access unless the wrapper is being improved.
- If an SDK-level test intentionally bypasses a wrapper, keep the hardware naming aligned with `RobotConfigurations` or the owning params class.
- The `pid/autoTune/` op modes are an intentional exception to the normal `Constants.create...(...)` factory rule: their direct motor-name configuration lives in `TuningConstants`, while the repo-local master adapters own the FTC relay-test integration.
- Keep subsystem creation on the `Constants` / `RobotConstants.createMechanism(...)` path; keep drive/follower profile data in `library/robot/RobotConstants`.
- Preserve FTC lifecycle expectations: initialize hardware before using it, update telemetry in the loop, and stop or persist state in `stop()` when needed.
- With the aggregate path, invoke `robot.start()` before its first periodic
  update and `robot.stop()` during cleanup. Use the autonomous mode explicitly
  so drive and intake periodic code does not consume teleop inputs.

## Examples in this repo

- Shared teleop base: `opModes/TeleOp.java`
- Ivy teleop: `opModes/IvyTeleOp.java`
- Iterative Ivy autonomous base and concrete example: `opModes/IvyAuton.java` and `opModes/variants/Test_IvyAuton.java`
- Linear Ivy autonomous helper: `pedroPathing/IvyAuto.java`
- Annotated teleop wrappers: `opModes/variants/Blue_TeleOp.java` and `opModes/variants/Red_TeleOp.java`
- Standalone validation op mode: `tests/MecanumTeleOp.java`
- Proof-of-concept test op mode: `tests/PreseasonWebcamAuto.java`
- Scratch iterative Ivy autonomous examples: `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt`
- FTC AutoTune calibration op modes: `pid/autoTune/PositionPIDTunerOpMode.java`, `pid/autoTune/VelocityPIDFTunerOpMode.java`, `pid/autoTune/DualVelocityPIDFTunerOpMode.java`
- Pre-start blackboard configuration menu: `configs/ConfigOpMode.java`

## Validation

- Build with Gradle before concluding changes are done.
- The root `build.gradle` uses Android Gradle Plugin `8.7.0`, and `TeamCode/build.gradle` also adds Lombok annotation processing plus Sloth load / runtime wiring, so build troubleshooting should include both files.
