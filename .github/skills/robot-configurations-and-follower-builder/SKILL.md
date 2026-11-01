---
name: robot-configurations-and-follower-builder
description: Use this skill for robot profiles, drivetrain wiring, Pinpoint configuration, and follower construction.
---

# Robot Configurations and Follower Builder

`library/robot/RobotConfigurations` holds static profile-builder methods that
each return a fully configured `RobotConstants`. Two profiles exist today:
`OldDECODERobotTesting()` (marked `// TODO DELETE THIS ONCE 1ST ROBOT BUILT`,
with measured `xVelocity`/`yVelocity` and tuned `FollowerConstants`/heading
PIDF/centripetal scaling) and `FirstRobot()` (has real motor names/directions
and Pinpoint pod offsets, but zero velocity constants, a default
`FollowerConstants()`, zero heading PIDF, and zero centripetal scaling —
placeholder tuning). `library/robot/RobotConstants` is the base class each
profile configures via a fluent builder API (`frontLeftMotorName(...)`,
`pinpointConstants(...)`, `driveConstants(...)`, `addMechanism(...)`, etc.) and
also owns `createFollower(HardwareMap)` / `createMechanism(HardwareMap, Class)`.

`Constants.currentBot = RobotConfigurations.FirstRobot()` selects the active
profile as a compile-time constant — docs must not describe this as freely
swappable at runtime; changing it means editing `Constants.java` and
rebuilding. Verify profile membership with `Constants.hasMechanism(Class)` /
`RobotConstants.Mechanisms.contains(Class)` rather than assuming every
mechanism is present.

## Factory facts

- `RobotConstants.createFollower(map)` assembles `FollowerConstants` (drive
  constants + heading PIDF + centripetal scaling), `MecanumConstants` (motor
  names/directions/max power/velocity), and `PinpointConstants` (profile pod
  offsets plus a forced `DistanceUnit.INCH` and `goBILDA_4_BAR_POD` encoder
  resolution), then builds a `PathConstraints` and passes everything to
  `new FollowerBuilder(...).pathConstraints(...).mecanumDrivetrain(...).pinpointLocalizer(...).build()`.
  It always builds a Pinpoint-based Pedro follower for the active profile.
- `Constants.createIMU(hardwareMap)` reads `currentBot.IMU_HARDWAREMAP_NAME`
  and initializes a `RevHubOrientationOnRobot` from
  `IMU_LOGO_FACING_DIRECTION`/`IMU_USB_FACING_DIRECTION`; it intentionally does
  not reset yaw.
- `Constants.createDrive/createIntake/createVision(hardwareMap)` each call
  `assertHasMechanism(Class)` (throws `IllegalStateException` if the profile
  never registered that mechanism) before delegating to
  `currentBot.createMechanism(hardwareMap, Class)`, which uses reflection to
  invoke the mechanism's `(HardwareMap, RobotConstants)` constructor and throws
  a wrapped `RuntimeException` on any construction failure.
- `Constants.createRobot(hardwareMap, RobotMode)` returns
  `new Robot(hardwareMap, currentBot, robotMode)`, which aggregates every
  mechanism whose class is in `currentBot.Mechanisms`. The no-arg
  `createRobot(hardwareMap)` overload defaults to `RobotMode.TELEOP` for
  teleop-style compatibility call sites.

## Registering mechanisms

Use `.configureDriveMechanisms()` (defaults to `new
RobotDrive.ROBOT_DRIVE_PARAMS()`) or `.configureDriveMechanisms(params)` to
register `RobotDrive`. Use `.addMechanism(Class)` for a mechanism with no
params override, or `.addMechanism(Class, params)` to both register the class
and store a copied params override (`RobotConstants.mechanismParams(...)`
calls `params.copy()` before storing, so later external mutation of the
builder-supplied instance cannot leak into the profile). Every
`Mechanism.Params<P>` implementation must implement real copy semantics
(`copy()` returning an independent instance) — avoid sharing one mutable
params object across multiple mechanisms or profiles.

`RobotConstants.getMechanismParams(Class, defaultParams)` returns a `.copy()`
of the stored override if one exists, otherwise a `.copy()` of the supplied
default — mechanisms always receive an isolated params instance through
`Mechanism.resolveParams(...)`.

## Calibration status

`FirstRobot()` has real physical wiring (motor names/directions, bot
width/length, Pinpoint hardware name and pod offsets, encoder reversal flags)
but not final follower tuning: zero `xVelocity`/`yVelocity`, a default
`FollowerConstants()` (no mass/zero-power-acceleration/PIDF overrides), zero
heading PIDF coefficients, and zero centripetal scaling. Treat those specific
fields as calibration work, not as validated match-ready tuning, even though
the rest of `FirstRobot()` reflects the actual robot.

## Reference profile: `OldDECODERobotTesting()`

`OldDECODERobotTesting()` is explicitly marked
`// TODO DELETE THIS ONCE 1ST ROBOT BUILT` and carries the previous season's
fully-measured numbers: `xVelocity`/`yVelocity` from empirical testing, a
`FollowerConstants` with `mass(11.4)`, measured zero-power accelerations,
tuned translational/heading PIDF coefficients, a `FilteredPIDFCoefficients`
drive PIDF, and `centripetalScaling(0.003)` — plus a different Pinpoint
hardware name (`"pinpoint"`) and IMU orientation (`LEFT`/`UP` vs.
`FirstRobot()`'s `UP`/`FORWARD`). Use it only as a worked example of a fully
tuned profile shape, not as data applicable to the current chassis; both
profiles register `RobotDrive`, `Intake` (reversed direction, hardware name
`"intake"`), and `Vision` with default params, and both set
`useFollowerDrive(true)`.

## `USE_FOLLOWER_DRIVE` fallback path

`RobotConstants.USE_FOLLOWER_DRIVE` (default `true`, set via
`.useFollowerDrive(...)`) controls whether `MecanumDrive` drives through the
Pedro follower's own teleop-drive API or falls back to directly commanding
four `Motor` wrappers built from the profile's motor names/directions
(`useManualDrive = !bot.USE_FOLLOWER_DRIVE`). The source comment notes the
team had reliability issues with follower-based teleop drive in a previous
season before getting it working — keep this flag in mind as an available
escape hatch if follower teleop drive misbehaves again, rather than assuming
manual drive motors are dead code.
