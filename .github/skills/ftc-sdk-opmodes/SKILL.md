---
name: ftc-sdk-opmodes
description: Use this skill when asked to create or edit FTC SDK op modes, wire hardware through HardwareMap, add TeleOp or Autonomous classes, or debug FTC lifecycle issues in this TeamCode project.
---

# FTC SDK Op Modes for This Repository

Use this skill when working on FTC SDK code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

## Repository context

- The app uses FTC SDK `11.1.0`.
- `TeamCode/` is the team module; `FtcRobotController/` is the controller app plus SDK samples.
- Shared robot constants and mechanism factories live in `Constants.java`, with drive-profile and follower configuration under `library/robot/`.
- Custom FTC `HardwareDevice` classes now live under `library/hardware/hardwareDevice/` for direct SDK registration and `hardwareMap.get(...)` usage.
- Pose and driver-preference handoff between op modes lives in `configs/Blackboard.java`.
- Shared match op modes live under `opModes/`, with `variants/` holding annotated wrappers such as `Blue_TeleOp` and `Red_TeleOp`.
- Validation and proof-of-concept op modes live under `tests/`.
- FTC AutoTune calibration op modes live under `pid/autoTune/`.
- One-shot configuration op modes live under `configs/`.
- Learning or scratch op modes live under `learning/`.
- Browser-side autonomous generators live under `java-autogen/`; they are tooling inputs to op-mode authoring, not selectable FTC classes by themselves.

## Use these patterns

1. Put reusable op mode logic in a shared base class under `opModes/`.
2. Put driver-station-visible wrappers in `opModes/variants/` and annotate those with `@TeleOp` or `@Autonomous`.
3. Resolve subsystem construction through `Constants.create...(...)` instead of direct constructors in op modes.
4. Let `Constants.create...(...)` delegate to the active robot profile when the subsystem is profile-managed.
5. If teleop needs the last autonomous pose or configured driver preference, read and write `configs.Blackboard` intentionally.
6. Use `tests/` for proof-of-concept or validation flows that are not part of the shared match architecture.
7. Use `pid/autoTune/` for driver-station-visible relay-feedback tuning utilities that use repo-local FTC master adapters backed by `pidautotuner-core`.
8. If an op mode wants loop timing or manual bulk-cache clearing, reuse `library/OpModeHelper.kt` instead of inventing a second helper with the same role.

## Central mechanism-creation convention

When an op mode needs a subsystem:

- call `Constants.createIntake(hardwareMap)` instead of `new Intake(...)`
- call `Constants.createLimelight(hardwareMap)` instead of `new Limelight(...)`
- call `Constants.createDrivetrain(hardwareMap)` or higher-level drive helpers instead of rebuilding drive hardware inline
- call `Constants.createIMU(hardwareMap)` instead of directly retrieving and orienting an `IMU`
- call `Constants.createRGB(hardwareMap)` or `Constants.createWebcam(hardwareMap)` when those profile-managed wrappers are needed

When adding a brand-new mechanism, add all three pieces together:

1. make it extend `Mechanism` if it should participate in the standard robot-subsystem flow
2. give it the `(HardwareMap, RobotConstants)` constructor expected by `RobotConstants.createMechanism(...)`
3. register it in `RobotConfigurations` with `.addMechanism(...)`
4. add a `Constants.create...(...)` helper if op modes should access it directly

The important repo contract is that profile-managed subsystems can be created uniformly from the active `RobotConstants` instance.

## Current repo conventions

- Teleop drive is usually routed through `MecanumDrive`.
- Intake is wrapped by `mechanisms/Intake.java` and created through `Constants`.
- Limelight usage is wrapped by `library/hardware/webcam/Limelight.java` and created through `Constants`.
- Logitech webcam and RGB indicator wrappers are also profile-managed and have `Constants` helpers.
- `mechanisms/Slides1.java` is the current slide mechanism shell, layered on top of `library/hardware/hardwareDevice/SlidesMotor.java`.
- Ivy-based op modes use `Scheduler.reset()`, `Scheduler.schedule(...)`, and `Scheduler.execute()`.
- `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt` are unannotated Java and Kotlin scratch iterative Ivy autonomous examples, not driver-station-visible match op modes.
- `learning/ExampleIvyWithSubsystems.java` is an unannotated scratch scaffold. Its unfinished `autoAlign()` step is commented out, and it initializes a direct intake `DcMotor` from `Intake.INTAKE_PARAMS`; do not use it as the profile-managed subsystem reference.
- Non-Ivy op modes call subsystem update methods directly inside `loop()` or the main `while (opModeIsActive())` block.
- `tests/MecanumTeleOp.java` is a current iterative FTC test op mode for raw mecanum drive, IMU yaw reset, and manual direction checks.
- `tests/MecanumTeleOp.java` now sources drivetrain motor names and directions from `Constants.currentBot` instead of separate test-only strings.
- `tests/MecanumTeleOp.java` and `tests/FieldCentricMecanumTeleOp.java` source IMU lookup and hub orientation from `Constants.createIMU(hardwareMap)`; yaw reset remains an explicit driver-control action.
- `tests/PreseasonWebcamAuto.java` is a proof-of-concept `LinearOpMode` that intentionally uses direct Limelight SDK access for color-based artifact tracking.
- `tests/PreseasonWebcamAuto.java` still uses raw `Limelight3A` access for behavior, but now pulls the camera name from `Limelight.LIMELIGHT_PARAMS`.
- `tests/MotorDirectionTest.java` is now a looping low-speed sanity check that keeps telemetry live while you flip drivetrain or intake directions.
- `pid/autoTune/PositionPIDTunerOpMode.java`, `VelocityPIDFTunerOpMode.java`, and `DualVelocityPIDFTunerOpMode.java` are `@TeleOp` calibration utilities in group `FtcAutoTune`; they use repo-local `PIDMaster` / `DualMotorPIDMaster` adapters plus `TuningConstants`, with `pidautotuner-core` providing the tuning primitives.
- `configs/ConfigOpMode.java` is the shared base for the blackboard-reset / driver-selection op modes.

## Rules

- Keep FTC annotations on concrete, selectable classes only.
- Prefer adding alliance-specific wrappers over duplicating whole op modes.
- Do not bypass mechanism wrappers with raw `DcMotorEx` or `Limelight3A` access unless the wrapper is being improved.
- If an SDK-level test intentionally bypasses a wrapper, keep the hardware naming aligned with `RobotConfigurations` or the owning params class.
- The `pid/autoTune/` op modes are an intentional exception to the normal `Constants.create...(...)` factory rule: their direct motor-name configuration lives in `TuningConstants`, while the repo-local master adapters own the FTC relay-test integration.
- Keep subsystem creation on the `Constants` / `RobotConstants.createMechanism(...)` path; keep drive/follower profile data in `library/robot/RobotConstants`.
- Preserve FTC lifecycle expectations: initialize hardware before using it, update telemetry in the loop, and stop or persist state in `stop()` when needed.

## Examples in this repo

- Shared teleop base: `opModes/TeleOp.java`
- Ivy teleop: `opModes/IvyTeleOp.java`
- Ivy autonomous base: `opModes/IvyAuton.java`
- Annotated wrappers: `opModes/variants/Blue_TeleOp.java`, `opModes/variants/Red_TeleOp.java`, `opModes/variants/Test_IvyAuton.java`
- Standalone validation op mode: `tests/MecanumTeleOp.java`
- Proof-of-concept test op mode: `tests/PreseasonWebcamAuto.java`
- Scratch iterative Ivy autonomous examples: `learning/ExampleIvyAuto.java` and `learning/kotlinTest/KtExampleIvyAuto.kt`
- FTC AutoTune calibration op modes: `pid/autoTune/PositionPIDTunerOpMode.java`, `pid/autoTune/VelocityPIDFTunerOpMode.java`, `pid/autoTune/DualVelocityPIDFTunerOpMode.java`
- One-shot config op mode family: `configs/ResetAllBlackboard.java`, `configs/ZeroPose.java`, `configs/Set*Driver.java`

## Validation

- Build with Gradle before concluding changes are done.
- This project uses Android Gradle Plugin `8.7.0`, so builds require Java 11 or newer.
- `TeamCode/build.gradle` now also includes Lombok annotation processing and Sloth load / runtime wiring, so build troubleshooting should include those pieces.
