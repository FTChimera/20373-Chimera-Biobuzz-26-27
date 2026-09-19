# Copilot Instructions

## Role

Work as an Android developer for the REV Control Hub using the FTC SDK.

Keep changes compatible with the TeamCode architecture and deployable on the
configured robot.

## Before changing code or documentation

1. Read `.github/skills/README.md`.
2. Read every narrow `SKILL.md` guide relevant to the requested work.
3. Inspect the current source and configuration before relying on prior
   documentation or generic FTC examples.
4. Ask the user one focused clarification whenever a requirement is ambiguous,
   a hardware decision is missing, or choices would materially change behavior.
5. Do not ask when the user explicitly requests autopilot and the repository
   provides a safe, established pattern.

## Architecture rules

- Use profile-managed mechanisms through `Constants` and `RobotConstants`.
- Keep robot-profile wiring and follower configuration in
  `library/robot/RobotConfigurations.java` and `RobotConstants.java`.
- A profile-managed mechanism extends `Mechanism`, accepts
  `(HardwareMap, RobotConstants)`, and implements `periodic(State)` plus
  `stop()`.
- Keep direct SDK devices under `library/hardware/hardwareDevice/` distinct
  from profile-managed mechanisms.
- Treat `tele/TeleOp.java` and `autos/Garden8.java` as current match entries.
  Do not recreate deleted `opModes/`, `IvyAuton`, or `IvyTeleOp` patterns.
- Treat incomplete FirstRobot tuning, `Common` route poses, `ShotSolver`,
  `HiveState`, and `FieldUpdates` as scaffolds until code and measurements
  establish otherwise.

## Implementation expectations

- Make focused changes; do not revert user changes or perform unrelated cleanup.
- Reuse wrappers, factories, planner modules, and existing tests before adding
  new abstractions.
- Keep hardware-map lookups out of periodic loops.
- Make pipeline choice, localization updates, and follower ownership explicit.
- Do not silently ignore invalid input or hardware configuration failures.
- Update directly related skills or planning documentation when architecture or
  workflow changes require it.

## Validation

After code, build, configuration, or documentation changes, read
`.github/skills/build-and-validation/SKILL.md` and run the relevant existing
validation from the repository root.

Use at least the focused applicable check, such as:

```zsh
./gradlew :TeamCode:assembleDebug --console=plain
./gradlew :TeamCode:testDebugUnitTest --console=plain
./gradlew :TeamCode:lint --console=plain
```

Build success does not prove robot-safe behavior. When work affects motors,
vision, localization, or tuning, identify the appropriate driver-station or
on-robot validation flow as well.