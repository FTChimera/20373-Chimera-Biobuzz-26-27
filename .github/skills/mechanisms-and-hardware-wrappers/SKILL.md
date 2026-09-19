---
name: mechanisms-and-hardware-wrappers
description: Use this skill for profile-managed mechanisms, reusable wrappers, and custom FTC hardware devices.
---

# Mechanisms and Hardware Wrappers

## Profile-managed contract

`mechanisms/Mechanism.java` is an abstract base:

```java
public abstract class Mechanism {
    public interface Params<T extends Params<T>> {
        T copy();
        T updateFromBot(RobotConstants bot);
    }
    protected HardwareMap hardwareMap;
    protected RobotConstants bot;
    public Mechanism(HardwareMap map, RobotConstants bot) { ... }
    protected <P extends Params<P>> P resolveParams(Class<? extends Mechanism> mechanismClass, P defaultParams) {
        return bot.getMechanismParams(mechanismClass, defaultParams).updateFromBot(bot);
    }
    public abstract void stop();
    public abstract void periodic(State robotState);
}
```

A profile-managed subsystem extends `Mechanism`, has a
`(HardwareMap, RobotConstants)` constructor (required by
`RobotConstants.createMechanism(...)`, which finds it via reflection), resolves
any nested `Params` implementation through `resolveParams(...)` so a profile
override is copied in, and implements both `periodic(State)` and `stop()` —
both are abstract, so a mechanism cannot compile without either one.

## Registration and aggregation

Register a mechanism class with `RobotConstants.addMechanism(Class, params)` or
the drivetrain-specific `.configureDriveMechanisms()` helper (both live on
`RobotConstants`, used fluently from `RobotConfigurations` profile builders). A
class only becomes constructible through `createMechanism(...)` once its
`Class` is present in `RobotConstants.Mechanisms`; otherwise both
`createMechanism(...)` and `Constants.assertHasMechanism(...)` throw.

`Robot` (itself a `Mechanism`) constructs every registered class in
`bot.Mechanisms` (sorted by class name) during its constructor, calls every
mechanism's `periodic(state)` during `Robot.periodic(state)`, and calls every
mechanism's `stop()` during `Robot.stop(...)` / `stopAllSubsystems()`. Look
mechanisms up with `Robot.get(Class)` (nullable), `Robot.require(Class)`
(throws `IllegalStateException` if missing), or the typed convenience methods
`getDrive()/requireDrive()`, `getIntake()/requireIntake()`,
`getVision()/requireVision()`. Use `Constants.create...(...)` factories or the
aggregate `Robot`, not a direct `new SomeMechanism(...)` constructor, in op
mode code.

## Current mechanisms

- `RobotDrive`: wraps `MecanumDrive`/`Follower`, tracks a `FollowerMode`
  (`MANUAL`, `FOLLOWING`, `HOLDING`, `TURNING`), and exposes
  `followPath(...)`, `holdPoint(...)`, `turn(...)`/`turnTo(...)`, and manual
  teleop drive. Its `periodic(State)` reads `ROBOT_DRIVE_PARAMS.TRANSLATION_STICK`
  /`ROTATION_STICK`/`ZERO_HEADING_BUTTON` from `GpBindings` only in `TELEOP`.
- `Intake`: trigger-controlled mechanism. `INTAKE_PARAMS` binds
  `FORWARD_COMMAND`/`REVERSE_COMMAND` (default `Trigger.RIGHT`/`Trigger.LEFT`),
  and `updateFromBot(bot)` disables itself (`ISBEINGUSED = false`) when
  `Intake.class` is not in the profile's `Mechanisms` set. Exposes both direct
  methods (`FullPower()`, `HalfPower()`, `ReverseFullPower()`, `Stop()`) and Ivy
  `Command` factories (`fullPower()`, `reverseFullPower()`, `stopCmd()`,
  `setPowerCommand(Supplier<Double>)`).
- `Vision`: Limelight pipelines and explicit relocalization support (see
  `limelight-wrapper-and-pipelines`).
- `Shooter`: flywheel scaffold. Its `SHOOTER_PARAMS.updateFromBot(...)` returns
  `null` (a latent bug if a future caller relies on it), and it optionally
  wraps a `DoubleCompositeController` (bang-bang + PIDF switch) built from
  `DoubleCompositeConstants` when `usingVelocityController` is set. Its
  `updateFlywheel(...)` switch over `ShooterMode` intentionally falls through
  `CRUISING` into `SPINNING_UP` (no `break`), so both branches call
  `flywheel.setVelocity(...)` back-to-back — treat as scaffold logic, not tuned
  behavior.
- `FieldUpdates`: game-state update scaffold. Both `stop()` and `periodic(State)`
  are empty bodies, and `getGoalPose()` unconditionally returns `null` — its
  goal-pose logic is not yet implemented.

`Shooter`, `FieldUpdates`, and their supporting `HiveState`/`ShotSolver` model
must be registered in a robot profile (they are not in `FirstRobot()` today)
before any `Robot.get/require` lookup can use them.

## Direct SDK devices vs. mechanisms

`library/hardware/hardwareDevice/{SlidesMotor,RevPotentiometer,RGBIndicator}`
are plain FTC device wrappers meant for direct `hardwareMap.get(...)` /
`new SlidesMotor(hardwareMap, name)` usage — they are not `Mechanism`
subclasses and are not aggregated by `Robot`. The former
`library/hardware/leds/RGBIndicator` location is now an empty directory; the
class lives only under `hardwareDevice/` today.

Keep low-level motor behavior in `Motor` (`library/hardware/actuators`),
profile data in `RobotConstants`, and mechanism behavior in the owning
mechanism class. `LogitechWebcam` is `@Deprecated`; new camera features belong
in `Vision`.
