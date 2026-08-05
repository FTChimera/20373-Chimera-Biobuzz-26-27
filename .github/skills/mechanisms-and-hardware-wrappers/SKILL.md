---
name: mechanisms-and-hardware-wrappers
description: Use this skill when asked to add or modify TeamCode wrappers, hardware factories, or subsystem classes under `library/hardware` or `mechanisms`.
---

# Mechanisms and Hardware Wrappers

Use this skill when extending the team’s abstraction layers instead of writing raw FTC SDK calls directly in op modes.

This includes both:

- profile-managed wrappers and subsystems built around `Mechanism`
- custom SDK-registerable hardware under `library/hardware/hardwareDevice/`

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConstants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/robot/RobotConfigurations.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Mechanism.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Motor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/RevPotentiometer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/SlidesMotor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/hardwareDevice/RGBIndicator.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/leds/RGBIndicator.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/LogitechWebcam.java`

## Design intent in this repo

1. Low-level device access belongs in wrappers such as `Motor`, `Limelight`, or `LogitechWebcam`.
2. Reusable subsystems and high-level wrappers share the `Mechanism` base class so they all receive `HardwareMap` and the active `RobotConstants` profile the same way.
3. Profile-controlled creation lives in `RobotConstants.createMechanism(...)`, with `Constants.create...(...)` acting as a convenience surface for common callers.
4. `Constants` now exposes dedicated helpers for common wrappers (`createIntake`, `createLimelight`, `createDrivetrain`, `createRGB`, `createWebcam`, `createTeleOpFollower`) plus a generic `createMechanism(...)` fallback.
5. Op modes should orchestrate mechanisms, not become the main place for hardware-map lookups, profile checks, or reflection-free one-off construction code.

The `OldDECODERobotTesting` and `FirstRobot` profiles exist. FirstRobot
registers only `MecanumDrive` and `Drivetrain`; its intake is planned but not
yet registered. Those helpers still depend on profile registration:
`Constants.assertHasMechanism(...)` runs before construction, so the selected
profile must register the requested class.

There is also a lower layer for custom FTC devices:

6. If a class is meant to be selectable via the FTC robot configuration UI with `hardwareMap.get(CustomType.class, ...)`, it belongs under `library/hardware/hardwareDevice/` and should implement or extend FTC `HardwareDevice` infrastructure directly.

## Required pattern for new mechanisms

When asked to create a new mechanism, follow this repo style:

1. Define the mechanism or wrapper class under `mechanisms/` or the appropriate `library/hardware/...` package.
2. Have it extend `Mechanism` if it participates in the robot profile / subsystem creation flow.
3. Give it a constructor shaped like `new NewMechanism(HardwareMap hardwareMap, RobotConstants bot)` and call `super(hardwareMap, bot)`.
4. Use `bot` for robot-profile state such as hardware names, directions, feature enablement, or follower compatibility.
5. Register the class in each relevant robot preset with `.addMechanism(NewMechanism.class)` or, for normal drive-stack behavior, through `configureDriveMechanisms()`.
6. Give it a builder-style nested params type that implements `Mechanism.Params<...>` when the mechanism has local configuration, and resolve that config through `Mechanism.resolveParams(...)`.
7. Implement the mechanism-wide `periodic(Bindings bindings)` API required by `Mechanism`; use a real per-loop update when the mechanism has one, otherwise keep the no-op implementation explicit.
8. Instantiate it through `RobotConstants.createMechanism(...)`, usually behind a small helper like `Constants.createNewMechanism(hardwareMap)` if higher-level code needs one.
9. If the mechanism can be disabled by profile selection, determine that before touching hardware.
10. Have op modes use the existing helper or `currentBot.createMechanism(...)` rather than constructing the class directly.

Why: the repo’s uniformity now comes from the shared `Mechanism` base class, profile registration with `addMechanism(...)`, and one reflection-based factory path in `RobotConstants.createMechanism(...)`.

## Existing patterns to reuse

- `Intake` extends `Mechanism`, owns a builder-style `INTAKE_PARAMS` helper, wraps the intake `Motor`, and now also performs raw `SlidesMotor` setup from its nested slides params without turning the slides into a separate profile-managed mechanism.
- `Mechanism` now requires every subclass to expose `periodic(Bindings bindings)`, where `Bindings` is a lightweight list of selected button readers from the gamepad wrapper layer.
- `Motor` is a low-level `DcMotorEx` wrapper that now implements `HardwareDevice`, and it also exposes helpers like `reverse()` and `getDirection()` that current drivetrain/intake direction tests rely on.
- `Motor.setDirection(Direction)` is the shared path for repo-level motor direction intent, with conversion to FTC SDK direction hidden inside the wrapper.
- `Limelight` extends `Mechanism`, keeps a builder-style `LIMELIGHT_PARAMS` instance resolved from the active profile, and is created via `Constants.createLimelight(hardwareMap)`.
- `SlidesMotor` is a custom `DcMotorImplEx`-based hardware device intended for direct `hardwareMap.get(...)` usage in SDK-style tests or other raw-device flows, and it now exposes convenience helpers like `setUp()`, `setDown()`, `setPositionPercentage(...)`, and `DEFAULT_HARDWAREMAP_NAME` on that raw-device layer.
- `RevPotentiometer` is a proof-of-concept custom analog sensor device that is meant to be fetched directly from `hardwareMap`.
- There are now two RGB-indicator layers: `library.hardware.hardwareDevice.RGBIndicator` is the custom FTC device type, while `library.hardware.leds.RGBIndicator` is the profile-managed wrapper/mechanism used by the rest of TeamCode.
- `RGBIndicator` extends `Mechanism`, owns builder-style `RGBIndicator_PARAMS`, now checks its enabled state before looking up the servo, and currently defaults to `Type.ONLY_HARDWARE` unless the params are explicitly changed.
- `LogitechWebcam` extends `Mechanism`, owns builder-style `LOGITECH_WEBCAM_PARAMS`, and now reads params before building `AprilTagProcessor` / `VisionPortal` hardware.
- `Drivetrain` extends `Mechanism`, owns `DRIVETRAIN_PARAMS`, and reads drive motor names/directions directly from the active `RobotConstants` profile.
- `MecanumDrive` extends `Mechanism`, owns `MECANUM_DRIVE_PARAMS`, combines a Pedro `Follower` with the team `Drivetrain` wrapper, and uses the active `RobotConstants` profile for both.
- `RobotConstants` now stores per-mechanism params overrides through `mechanismParams(...)` / `getMechanismParams(...)`, so builder-style nested params can vary by robot profile without moving all subsystem data into the robot profile class.
- Profile params supplied through `mechanismParams(...)` do not make a class factory-eligible; the profile selected by `Constants.currentBot` must also register it with `addMechanism(...)`.

## Rules

- Add new reusable hardware access under `library/hardware/...` first.
- Add custom FTC robot-configuration devices under `library/hardware/hardwareDevice/...` when the goal is `hardwareMap.get(CustomType.class, ...)` support.
- Add robot behavior under `mechanisms/...` second.
- For a profile-managed subsystem, add the `Mechanism` subclass, register it in `RobotConfigurations`, and expose a `Constants.create...(...)` helper when it should be easy to consume.
- If the wrapper is optional, do not fetch hardware until its params or profile say it is in use.
- Preserve chainable convenience methods where a wrapper already uses them, as `Motor` does.
- If a wrapper or profile needs reusable drive direction state, prefer the shared `Direction` enum instead of pushing `DcMotorSimple.Direction` through higher-level config.
- If a mechanism already exposes Ivy command helpers, prefer extending that API instead of duplicating inline commands in op modes.
- Make sure any class created through `RobotConstants.createMechanism(...)` actually has the required `(HardwareMap, RobotConstants)` constructor.

## Good edits for common tasks

- New motor behavior like brake/float/PIDF convenience: `Motor.java`
- New intake-level operation or params field: `Intake.java`
- New robot-profile registration or creation eligibility: `RobotConfigurations.java` / `RobotConstants.java`
- New top-level subsystem convenience helper: `Constants.java`
- New LED indicator behavior or params: `RGBIndicator.java`
- New custom FTC device registration or controller/port behavior: `library/hardware/hardwareDevice/...`
- New slides raw-device behavior: `SlidesMotor.java`
- New drive behavior or teleop helper: `Drivetrain.java` or `MecanumDrive.java`
- New camera wrapper behavior: `Limelight.java` or `LogitechWebcam.java`

## Things to avoid

- Bypassing `Motor` with raw `DcMotorEx` in an op mode unless the wrapper itself needs improvement.
- Bypassing `Limelight` with raw `Limelight3A` calls all over the codebase.
- Instantiating the same mechanism differently in every op mode.
- Forgetting to add a new mechanism class to `RobotConfigurations`; `RobotConstants.createMechanism(...)` intentionally refuses to create classes that are not registered in the active profile.
- Forgetting that `Constants.assertHasMechanism(...)` now enforces profile registration on the convenience-factory path too.
- Confusing the device-level `library.hardware.hardwareDevice.RGBIndicator` with the profile-managed `library.hardware.leds.RGBIndicator` wrapper.
- Putting a custom FTC device in the `Mechanism` factory flow when it is really meant to be used directly through `hardwareMap.get(...)`.
