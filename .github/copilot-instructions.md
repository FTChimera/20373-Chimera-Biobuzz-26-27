# Copilot instructions for 20373-Chimera-Biobuzz-26-27

## Build, test, and lint

- **Java requirement:** Gradle must run on **Java 11+**. This repo uses Android Gradle Plugin **8.7.0**; Java 8 fails during configuration before TeamCode compiles.
- **Build TeamCode:** `./gradlew :TeamCode:assembleDebug`
- **Build whole project:** `./gradlew assembleDebug`
- **Lint TeamCode:** `./gradlew :TeamCode:lint`
- **Unit tests:** `./gradlew :TeamCode:testDebugUnitTest`
- **Single unit test:** `./gradlew :TeamCode:testDebugUnitTest --tests 'fully.qualified.TestClassName'`

There is currently no dedicated `TeamCode/src/test` or `TeamCode/src/androidTest` suite checked in. Most behavior validation in this repo happens through driver-station-visible test and tuning op modes such as `tests/MecanumTeleOp`, `tests/MotorDirectionTest`, `tests/PedroPathingTeleOpTest`, and `pedroPathing/Tuning`.

## High-level architecture

- `Constants.java` is the top-level entry point for robot code. It selects the active robot profile with `RobotConfigurations.OldDECODERobotTesting()` and exposes op-mode-facing factories like `createFollower(...)`, `createTeleOpFollower(...)`, `createIntake(...)`, `createLimelight(...)`, and `createDrivetrain(...)`.
- `library/robot/RobotConstants.java` is the real robot profile object. It holds drivetrain names/directions, Pedro follower constants, Pinpoint localizer constants, path constraints, and the enabled mechanism list. It also owns `createFollower(...)` and the reflective `createMechanism(...)` factory used across the repo.
- `library/robot/RobotConfigurations.java` is where a specific robot preset is assembled. It uses builder-style setters on `RobotConstants` and registers optional subsystems with `.addMechanism(...)`.
- `mechanisms/` contains reusable subsystems. `Mechanism` is the shared base class, and profile-managed mechanisms are instantiated through `RobotConstants.createMechanism(...)`. `MecanumDrive` is the main drive abstraction: it wraps a Pedro `Follower`, also composes the manual `Drivetrain`, and switches between follower-driven and manual teleop with `USE_FOLLOWER_DRIVE`.
- `opModes/` contains shared op mode logic. `TeleOp` is the non-annotated base teleop flow; `IvyAuton` is the non-annotated autonomous base that schedules Pedro Ivy commands. `opModes/variants/` holds the thin FTC-annotated wrappers that appear on the Driver Station.
- `configs/Blackboard.java` is the cross-opmode state store. It currently carries the shared `POSE` handoff and selected `DRIVER`, and small config op modes under `configs/` mutate that state.
- `pedroPathing/` contains Pedro-specific integration: tuning op modes, Ivy helpers, command adapters, custom localizers, and custom geometry like `custom/EquationCurve`.
- `library/` contains lightweight shared types and wrappers used by the higher layers, including `DriveVector`, actuator wrappers, gamepad wrappers, timers, enums, and math helpers.

## Key conventions

- Prefer the repo factory flow over direct construction in op modes. Shared code should normally call `Constants.create...(...)`, which routes through the active `RobotConstants` profile.
- New profile-managed subsystems should extend `mechanisms.Mechanism`, use a constructor shaped like `(HardwareMap, RobotConstants)`, and be registered in `RobotConfigurations` with `.addMechanism(...)`.
- Keep robot-profile-wide configuration in `library/robot/RobotConstants` and `RobotConfigurations`: motor names, `Direction` values, follower tuning, Pinpoint setup, path constraints, and the enabled-mechanism list belong there.
- Keep `Constants.java` small and top-level: active profile selection, default poses, and convenience factories belong there; mechanism-specific behavior does not.
- Preserve pose handoff through `Blackboard`. `IvyAuton.stop()` stores the follower pose, and the alliance teleop variants restore from `Blackboard.Caption.POSE` before falling back to the default start pose.
- Driver preferences are also stored in `Blackboard`. `Driver` controls whether teleop drive defaults to field-centric, and both `Drivetrain` and `MecanumDrive` read that shared setting.
- `opModes/variants/` should stay thin wrappers around shared base classes. Put orchestration in `opModes/`, not duplicated inside each annotated variant.
- The shared actuator/config boundary uses the repo’s `Direction` wrapper, then converts to FTC SDK directions at the hardware/follower boundary.
- `@Configurable` classes are intentionally used with the Panels tooling for runtime tuning; do not remove that wiring casually.
- `TeamCode/build.gradle` intentionally includes Lombok annotation processing plus Dairy Foundation Sloth load/runtime support. Treat both as part of the expected TeamCode build wiring.
