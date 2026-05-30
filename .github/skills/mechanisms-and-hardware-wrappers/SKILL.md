---
name: mechanisms-and-hardware-wrappers
description: Use this skill when asked to add or modify TeamCode wrappers, hardware factories, or subsystem classes under `library/hardware` or `mechanisms`.
---

# Mechanisms and Hardware Wrappers

Use this skill when extending the team’s abstraction layers instead of writing raw FTC SDK calls directly in op modes.

## Relevant files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Direction.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Drivetrain.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/actuators/Motor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/Limelight.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/library/hardware/webcam/LogitechWebcam.java`

## Design intent in this repo

1. Low-level device access belongs in wrappers such as `Motor`, `Limelight`, or `LogitechWebcam`.
2. Subsystem behavior belongs in mechanisms such as `Intake`, `Drivetrain`, and `MecanumDrive`.
3. Shared creation points belong in `Constants`, not a separate `Mechanisms` factory class.
4. Op modes should orchestrate mechanisms, not become the main place for hardware-map lookups, device math, or params construction.

## Required pattern for new mechanisms

When asked to create a new mechanism, follow this repo style:

1. Define the mechanism class under `mechanisms/`.
2. Give it a nested params type, usually named for the subsystem, such as `INTAKE_PARAMS`, `LIMELIGHT_PARAMS`, or a new `MECHANISM_PARAMS`.
3. Pass that params object into the constructor: `new NewMechanism(hardwareMap, params)`.
4. Store the params instance on the mechanism object, for example `public final MECHANISM_PARAMS params;`.
5. Create one shared editable params instance in `Constants`.
6. Add a `Constants.createNewMechanism(hardwareMap)` factory that injects that shared params instance.
7. Have op modes call the `Constants` factory instead of constructing the mechanism directly.

Why: the team wants Dashboard/panels to edit subsystem params from one central place. That only works well if the shared params instance lives in `Constants` instead of being recreated or hidden statically inside each mechanism.

## Existing patterns to reuse

- `Intake` wraps one `Motor`, keeps an `INTAKE_PARAMS` instance, and is created via `Constants.createIntake(hardwareMap)`.
- `Motor.setDirection(Direction)` is the shared path for repo-level motor direction intent, with conversion to FTC SDK direction hidden inside the wrapper.
- `Limelight` keeps a `LIMELIGHT_PARAMS` instance and is created via `Constants.createLimelight(hardwareMap)`.
- `Drivetrain` is created via `Constants.createDrivetrain(hardwareMap)` using the active `RobotConstants` profile.
- `MecanumDrive` combines a Pedro `Follower` with the team `Drivetrain` wrapper and delegates follower/drivetrain creation through `Constants`.

## Rules

- Add new reusable hardware access under `library/hardware/...` first.
- Add robot behavior under `mechanisms/...` second.
- Add the central shared params instance and factory in `Constants` as part of the same feature.
- Preserve chainable convenience methods where a wrapper already uses them, as `Motor` does.
- If a wrapper or profile needs reusable drive direction state, prefer the shared `Direction` enum instead of pushing `DcMotorSimple.Direction` through higher-level config.
- If a mechanism already exposes Ivy command helpers, prefer extending that API instead of duplicating inline commands in op modes.
- Do not create a second static params singleton inside the mechanism class.

## Good edits for common tasks

- New motor behavior like brake/float/PIDF convenience: `Motor.java`
- New intake-level operation or params field: `Intake.java`
- New central subsystem factory or shared params instance: `Constants.java`
- New drive behavior or teleop helper: `Drivetrain.java` or `MecanumDrive.java`
- New camera wrapper behavior: `Limelight.java` or `LogitechWebcam.java`

## Things to avoid

- Bypassing `Motor` with raw `DcMotorEx` in an op mode unless the wrapper itself needs improvement.
- Bypassing `Limelight` with raw `Limelight3A` calls all over the codebase.
- Instantiating the same mechanism differently in every op mode.
- Hiding Dashboard-editable params inside per-opmode constructors instead of centralizing them in `Constants`.

