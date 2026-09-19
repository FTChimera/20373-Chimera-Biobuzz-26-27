---
name: ftc-sdk-hardwaremap-and-lifecycle
description: Use this skill for FTC SDK HardwareMap access and op mode lifecycle work.
---

# FTC SDK HardwareMap and Lifecycle

Use FTC SDK lifecycle callbacks correctly and keep hardware construction profile-managed.

## Match entries

- `tele/TeleOp` extends `LinearOpMode`.
- It uses `runOpMode()`, `waitForStart()`, and `while (opModeIsActive())`.
- `autos/Garden8` extends iterative `OpMode`.
- It implements `init()`, `start()`, `loop()`, and `stop()`.
- Its nested red and blue classes are the annotated autonomous registrations.
- `TeleOp` contains the annotated base entry plus nested red and blue selections.

## Hardware construction

- Pass the FTC SDK `hardwareMap` to `Constants.createRobot(...)`.
- The aggregate `Robot` uses the active `RobotConstants` profile.
- Profiles choose registered mechanisms and their device construction.
- Do not instantiate motors, cameras, or the aggregate robot with guessed names in an entry point.
- Confirm a mechanism is registered before calling a `require...()` accessor.
- Use an optional getter when an entry intentionally supports missing hardware.

## Linear teleop sequence

1. Construct `DualGamepad(gamepad1, gamepad2)`.
2. Create `State(RobotMode.TELEOP)`.
3. Create the robot with the same mode.
4. Set starting pose, alliance, drive curve, and teleop drive mode.
5. Put bindings on the state.
6. Show initialization telemetry.
7. Wait for start.
8. Start the robot and selected vision pipeline.
9. Update gamepads and call `robot.periodic(state)` each active loop.
10. Always call `robot.stop()` on exit.

## Iterative autonomous sequence

1. Reset the Ivy scheduler in `init()`.
2. Create the aggregate robot and build the autonomous command.
3. Start the robot in `start()`.
4. Execute the scheduler in `loop()`.
5. Apply time-limit and parking behavior in the loop.
6. Stop the aggregate robot exactly once.
7. Reset the scheduler in `stop()`.

## Mechanism lifecycle

- Every profile-managed mechanism extends `Mechanism`.
- Every such mechanism provides `periodic(State)` and `stop()`.
- `Robot.periodic(State)` updates its helper and state context before dispatch.
- It rejects a state whose mode differs from its configured mode.
- `Robot.stop()` stops all configured mechanisms and closes helper resources.
- Autonomous shutdown can save pose and alliance through `saveEndVals()`.

## Vision lifecycle

- `Robot.start()` attempts to start the AprilTag pipeline when Vision is configured.
- `Robot.startVision(pipeline)` is safe when Vision is absent.
- `Robot.stopVision()` is likewise optional.
- A direct `requireVision()` call is a declared requirement.
- Do not assume vision updates drive pose automatically.
- Request relocalization explicitly when a valid vision measurement should affect pose.

## SDK safety

- Never access hardware before FTC initialization has provided `hardwareMap`.
- Keep blocking work out of iterative `loop()`.
- In a linear mode, honor interruption and stop requests.
- Stop motors and close resources through the robot lifecycle rather than scattered cleanup.
- Avoid static hardware instances and retain only configuration-level static data.

## Validation

- Compile with `./gradlew :TeamCode:assembleDebug`.
- Use existing test op modes for physical device verification.
- Verify device names against the active Control Hub configuration.
- Verify drivers can start and stop both match entries cleanly.
- Confirm autonomous pose handoff is available to the following teleop.

## Failure handling

- Let FTC lifecycle methods return promptly when a stop is requested.
- Use telemetry to expose initialization failures before match start.
- Do not suppress missing required hardware errors with broad exception handling.
- For optional hardware, choose the optional aggregate accessor deliberately.
- Avoid closing an SDK device twice through both a mechanism and an entry point.
- Reset scheduler-owned work when an iterative autonomous stops early.
- Keep explicit cleanup near aggregate `Robot.stop()` ownership.
- Verify a changed lifecycle path on hardware before competition use.

## Change boundaries

- Keep FTC callback code concise and delegate robot behavior to the aggregate.
- Keep profile device lookup out of annotations and static initializers.
- Keep scheduler execution within the iterative lifecycle that owns it.
- Recheck all exit paths after adding a new callback or early-return condition.
