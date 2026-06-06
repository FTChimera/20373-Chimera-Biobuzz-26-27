---
name: mechanisms-and-hardware-wrappers
description: Use this skill when asked to add or modify TeamCode wrappers, hardware factories, or subsystem classes under `library/hardware` or `mechanisms`.
---

# Mechanisms and Hardware Wrappers

Use this skill when extending the team’s abstraction layers instead of writing raw FTC SDK calls directly in op modes.

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
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/LogitechWebcam.java`

## Design intent in this repo

1. Low-level device access belongs in wrappers such as `Motor`, `Limelight`, or `LogitechWebcam`.
2. Reusable subsystems and high-level wrappers share the `Mechanism` base class so they all receive `HardwareMap` and the active `RobotConstants` profile the same way.
3. Profile-controlled creation lives in `RobotConstants.createMechanism(...)`, with `Constants.create...(...)` acting as a convenience surface for common callers.
4. Op modes should orchestrate mechanisms, not become the main place for hardware-map lookups, profile checks, or reflection-free one-off construction code.

## Required pattern for new mechanisms

When asked to create a new mechanism, follow this repo style:

1. Define the mechanism or wrapper class under `mechanisms/` or the appropriate `library/hardware/...` package.
2. Have it extend `Mechanism` if it participates in the robot profile / subsystem creation flow.
3. Give it a constructor shaped like `new NewMechanism(HardwareMap hardwareMap, RobotConstants bot)` and call `super(hardwareMap, bot)`.
4. Use `bot` for robot-profile state such as hardware names, directions, feature enablement, or follower compatibility.
5. Register the class in each relevant robot preset with `.addMechanism(NewMechanism.class)`.
6. Instantiate it through `RobotConstants.createMechanism(...)`, usually behind a small helper like `Constants.createNewMechanism(hardwareMap)` if higher-level code needs one.
7. Have op modes use the existing helper or `currentBot.createMechanism(...)` rather than constructing the class directly.

Why: the repo’s uniformity now comes from the shared `Mechanism` base class, profile registration with `addMechanism(...)`, and one reflection-based factory path in `RobotConstants.createMechanism(...)`.

## Existing patterns to reuse

- `Intake` extends `Mechanism`, wraps one `Motor`, owns an `INTAKE_PARAMS` helper, and is created via `Constants.createIntake(hardwareMap)` → `currentBot.createMechanism(...)`.
- `Motor.setDirection(Direction)` is the shared path for repo-level motor direction intent, with conversion to FTC SDK direction hidden inside the wrapper.
- `Limelight` extends `Mechanism`, keeps a `LIMELIGHT_PARAMS` instance, and is created via `Constants.createLimelight(hardwareMap)`.
- `Drivetrain` extends `Mechanism` and reads drive motor names/directions directly from the active `RobotConstants` profile.
- `MecanumDrive` extends `Mechanism`, combines a Pedro `Follower` with the team `Drivetrain` wrapper, and delegates follower/drivetrain creation through `Constants`.

## Rules

- Add new reusable hardware access under `library/hardware/...` first.
- Add robot behavior under `mechanisms/...` second.
- For a profile-managed subsystem, add the `Mechanism` subclass, register it in `RobotConfigurations`, and expose a `Constants.create...(...)` helper when it should be easy to consume.
- Preserve chainable convenience methods where a wrapper already uses them, as `Motor` does.
- If a wrapper or profile needs reusable drive direction state, prefer the shared `Direction` enum instead of pushing `DcMotorSimple.Direction` through higher-level config.
- If a mechanism already exposes Ivy command helpers, prefer extending that API instead of duplicating inline commands in op modes.
- Make sure any class created through `RobotConstants.createMechanism(...)` actually has the required `(HardwareMap, RobotConstants)` constructor.

## Good edits for common tasks

- New motor behavior like brake/float/PIDF convenience: `Motor.java`
- New intake-level operation or params field: `Intake.java`
- New robot-profile registration or creation eligibility: `RobotConfigurations.java` / `RobotConstants.java`
- New top-level subsystem convenience helper: `Constants.java`
- New drive behavior or teleop helper: `Drivetrain.java` or `MecanumDrive.java`
- New camera wrapper behavior: `Limelight.java` or `LogitechWebcam.java`

## Things to avoid

- Bypassing `Motor` with raw `DcMotorEx` in an op mode unless the wrapper itself needs improvement.
- Bypassing `Limelight` with raw `Limelight3A` calls all over the codebase.
- Instantiating the same mechanism differently in every op mode.
- Forgetting to add a new mechanism class to `RobotConfigurations`; `RobotConstants.createMechanism(...)` intentionally refuses to create classes that are not registered in the active profile.

