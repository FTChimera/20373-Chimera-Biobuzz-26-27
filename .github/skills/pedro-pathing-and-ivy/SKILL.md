---
name: pedro-pathing-and-ivy
description: Use this skill when asked to add or tune Pedro Pathing followers, paths, localizers, or Ivy scheduler commands in this FTC repository.
---

# Pedro Pathing and Ivy in This Repository

This repository uses Pedro Pathing for localization, teleop drive support, and autonomous path following, plus Pedro Ivy for command scheduling.

## Key files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/custom/LimelightPinpointLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/IvyAuton.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes/variants/Test_IvyAuton.java`

## How this repo is wired

1. Follower creation is centralized in `Constants`.
2. `Constants.pinpointFollower(...)` is the default follower for most drive and autonomous code.
3. `Constants.customLocalizerFollower(...)` swaps in `LimelightPinpointLocalizer`.
4. `MecanumDrive` wraps a Pedro `Follower` plus the team `Drivetrain`.
5. Ivy-based autonomous code usually defines an `IvyAuton_Paths` implementation in a variant class and passes it into `IvyAuton`.

## Use these patterns

1. Add follower or localizer changes in `Constants`, not scattered across op modes.
2. Put autonomous geometry in a variant-local `Paths` class that extends `IvyAuton.IvyAuton_Paths`.
3. Return a `PathChain` from autonomous path builders, even for a single `Path`.
4. For teleop with Ivy, schedule subsystem periodic commands and long-running control commands through the scheduler.
5. For non-Ivy pathing code, call `follower.update()` every loop.

## Ivy command construction

Use one of these three command patterns when adding new Ivy behavior:

1. Implement `Command` directly when the command has custom lifecycle behavior and should own all of `requirements()`, `priority()`, `start()`, `execute()`, `done()`, and `end(...)`.
2. Use `new CommandBuilder()` for most inline or one-off commands.
3. Extend `CommandBuilder` when you want a reusable base command with builder-style configuration.

`CommandBuilder` already implements `Command`, so the normal pattern is to configure it with:

- `setStart(...)`
- `setExecute(...)`
- `setDone(...)`
- `setEnd(...)`
- `requiring(...)`
- `setPriority(...)`
- `setInterruptedBehavior(...)`
- `setBlockedBehavior(...)`
- `setConflictBehavior(...)`

Prefer `new CommandBuilder()` for lightweight commands that are more complex than a simple `Runnable`, and prefer implementing `Command` or extending `CommandBuilder` for reusable command types.

Examples:

```java
Command command = new CommandBuilder()
        .setStart(() -> intake.setPower(1.0))
        .setDone(() -> false)
        .setExecute(drive::periodic)
        .requiring(intake, drive);
```

```java
public final class IntakeCommand extends CommandBuilder {
    public IntakeCommand(Intake intake, double power) {
        setStart(() -> intake.setPower(power));
        setDone(() -> true);
        requiring(intake);
    }
}
```

## Important repo-specific constraints

- `Constants` still contains placeholder tuning values for follower, drivetrain, and pinpoint offsets; do not assume those numbers are competition-ready.
- `customLocalizerFollower(...)` uses Limelight plus Pinpoint fusion. The source comment explicitly warns not to use it when the camera is needed for color detection.
- `Tuning.java` is the Pedro tuning entry point and already uses `Constants.pinpointFollower(...)`.
- `MecanumDrive.pinpointLocalizer(...)` is the normal teleop entry point.

## Rules

- Reuse the existing `FollowerBuilder` flow in `Constants` instead of constructing ad hoc followers elsewhere.
- Keep localization decisions explicit: choose Pinpoint-only or Limelight-plus-Pinpoint based on the active vision pipeline.
- Do not add duplicate drive update loops; there should be one consistent source of `follower.update()`.
- Prefer Pedro/Ivy commands for path following in Ivy op modes instead of custom busy loops.

## Example workflow

1. Define or update path geometry in an `IvyAuton_Paths` subclass.
2. Keep start pose and `PathChain` together.
3. In `IvyAuton.start()`, compose behavior with `Groups.sequential(...)`.
4. Stop the op mode through `CommandConverter.toCommand(this::requestOpModeStop)` when the sequence finishes.
