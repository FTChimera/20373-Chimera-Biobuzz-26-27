# 20373 2026-27 Robot Code - Chimera

![20373 RMS Chimera logo](https://raw.githubusercontent.com/FTChimera/FTChimera/main/logoWithText.png)

This is the code for FTC team 20373's 2026-27 competition robot Chimera, built for the BIOBUZZ season.

Our first qualifier is November 21, 2026 at San Jose #1 (Bellarmine College Prep). [FtcScout](https://ftcscout.org/teams/20373?season=2026)

![Chimera](./chimera.jpg)

[`MainTeleOp.java`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java) is the main driver-controlled op mode. It manages input from two drivers, handling drivetrain movement, the two-motor shooter, intaking, transfer, the deployable ramp, and auto-aiming.

The [`auto/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto) folder contains the autonomous programs: Garden 8 and Loading 8, two 8-ball routines. We use [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing) for smooth, accurate path following and run through intake-shoot cycles to score as many balls as possible before the driver-controlled period begins.

## Two iterations

Chimera has been through two builds this season:

- **v1 — pollen + nectar.** The first version shot both ball types, built for the RMS FTC Scrimmage. The old code lives on the [`old`](https://github.com/FTChimera/20373-Chimera-Biobuzz-26-27/tree/old) branch.
- **v2 — pollen-only, custom drivetrain.** We rebuilt the robot around a custom drivetrain and narrowed the shooter to pollen only. This is the version in this repo.

## Under the hood

A few of the ideas doing the heavy lifting:

- **A\* search for parking.** The endgame park route is planned by [A\*](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/game/AStarPathfinder.java) over a 6-inch cell grid of the whole field (24x24): 8-direction moves with no diagonal corner-cutting, an octile heuristic, line-of-sight-checked steps, and a smoothing pass that collapses the grid staircase into straight legs around forbidden space — the hive bases and the flowers — using the [JTS Topology Suite](https://github.com/locationtech/jts) for the geometry.
- **Kalman-filter localization.** [`FusionLocalizer`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/FusionLocalizer.java) fuses dead reckoning with measurements through a Kalman filter (covariance tracking, history buffer), and [`util/control/filters/`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/filters) holds a hand-written extended Kalman filter for nonlinear models.
- **PIDSVA flywheel control, tuned automatically.** The two-motor shooter runs a full velocity controller — PID plus feedforward (kS/kV/kA) with voltage compensation — and its constants come from a custom PIDSVA autotuner (relay test + Ziegler-Nichols + least-squares fit), built on [Aarav Dhawan's FtcAutoTune](https://github.com/aaravdhawan25/FtcAutoTune).
- **Command-based autonomous.** Autos are composed from [BehaviorArchitectures](https://github.com/shlok-k720/BehaviorArchitectures) commands (`Plan.sequence` + `PathPlanner`), so each routine is a chain of small reusable steps: drive a path, intake, line up on the hive, shoot, park.

## Libraries

- [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing) 3.0.2 (+ tuning 1.0.1) — path following and localization
- [FtcAutoTune](https://github.com/aaravdhawan25/FtcAutoTune) 1.4.3 — PID/PIDF autotuning (relay + Ziegler-Nichols)
- [BehaviorArchitectures](https://github.com/shlok-k720/BehaviorArchitectures) 0.2.2 — our own behavior-tree, command, and finite-state-machine library (drives all competition OpModes)
- [JTS Topology Suite](https://github.com/locationtech/jts) 1.20.0 — computational geometry for field and obstacle math
- [Dairy](https://repo.dairy.foundation) Sloth 0.3.2 + FTC Dashboard — telemetry and live tuning
- FTC SDK 12.0.0

## Codebase organization

All robot code lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

```
config/       Robot configuration: alliance color, start poses, hardware names
mechanisms/   The subsystems: Robot, Shooter (two motors), Intake, Transfer, Vision
opmodes/      MainTeleOp, Controls, and auto/ (Garden 8, Loading 8, ParkAuto)
pedro/        Pedro Pathing setup: Follower, Constants, FusionLocalizer, Tuning
planners/     Autonomous planning: Plan, PathPlanner, field poses (Common.kt)
robotDev/     Development and tuning tools: AutoTuner, subsystem tests
util/         Shared building blocks, grouped by job:
  autotune/   Relay tuner, Ziegler-Nichols, S/V/A fit
  control/    PID/PIDF controllers, filters (incl. extended Kalman filter)
  game/       Field model, hive state, A* pathfinder
  hardware/   SmartFlywheel, cached motors, gamepad helpers
  shotcalc/   Shot tables and the ideal shot model (per motor)
  ...plus math, poses, interpolation, localization
```

## Deterministic FTC JVM tests

The JUnit 4 framework lives entirely in `TeamCode/src/test/java`: fake SDK devices
are not packaged in the Control Hub APK. It takes inspiration from
[PsiLynx/RobotCore's JVM simulation](https://github.com/PsiLynx/RobotCore/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sim),
with explicit hardware registration, per-test state isolation, bounded OpMode
execution, and assertions on real control logic rather than permissive stubs.

```sh
./gradlew :TeamCode:testDebugUnitTest --console=plain
./gradlew :TeamCode:testDebugUnitTest --tests '*fakeHw.*' --tests '*testing.*' --tests '*SimulationTest' --console=plain
./gradlew :TeamCode:assembleDebug --console=plain
```

In Android Studio, sync Gradle and select **Unit Test All** (the Gradle run
configuration) to launch the existing `UnitTestAll` suite. For individual tests,
set **Settings > Build, Execution, Deployment > Build Tools > Gradle > Run tests
using** to **Gradle** where available. The unfiltered Gradle task above discovers
all test classes, including the simulation examples.

If the IDE reports `NoClassDefFoundError: junit/textui/TestRunner` and its launch
classpath contains only IDE runners and `android.jar`, it is using an incomplete
direct JUnit module classpath. Use the shared Gradle launcher rather than adding
JUnit to the robot's `implementation` dependencies.

| Boundary | Desktop substitute |
| --- | --- |
| FTC hardware | `fakeHw/` stateful motors/controllers, servos, CR servos, voltage, analog/digital, color/distance sensors and IMU |
| Hardware configuration | `FakeHardwareMap`: explicitly register names; missing devices, wrong types and duplicate names are errors |
| Time | `FakeClock`, injectable robot/controller/command timers, and scoped `PedroTimerScope` for Pedro's non-injectable timer |
| Driver Station | `OpModeRunner`, programmable gamepad fields, and `RecordingTelemetry` |
| Pinpoint/JNI localization | `SimulatedMecanumLocalizer`, integrating measured simulated wheel speeds; real Pedro Foresight and Mecanum remain in use |
| Limelight and Prism | Existing `Vision.stub` / `Lights.stub` simulation factories; no camera detections or LED-driver I/O |
| Android dashboard/storage | Existing simulation guards and in-memory storage, enabled and restored by `FtcSimulationRule` |
| Android/SDK logging | `AndroidLogScope` records log entries and restores the real boundary when closed |

Use `@Rule public final FtcSimulationRule simulation = new FtcSimulationRule();`
for robot tests. It enables desktop services before storage initialization and
restores alliance/pose/match state, SOTM, follower settings
and environment flags even when a test fails. Advance `simulation.clock` and device physics explicitly;
never use wall-clock sleeps to wait for a controller to settle. Mockito is a
**test-only** dependency for final SDK lifecycle/time methods and vendor timers;
mechanisms, commands, shot calculations and path-control algorithms remain real.
Unsupported boundaries should fail explicitly, not silently return success.
`Robot.update()` synchronously advances timing, battery, queued drive, vision,
hive state, aiming, follower, smart controls, mechanisms and lights, then optional
debug telemetry; call `start()` first and do not update after `stopEverything()`.
Use `hardwareMap.register(name, device)` to populate both general lookups and
SDK `DeviceMapping`s (especially `voltageSensor`); SDK-style `put()` alone is
not a substitute for complete registration. Wrap tests constructing real SDK
devices that log in `try (AndroidLogScope logs = new AndroidLogScope())`.

`OpModeRunner` executes the real `runOpMode()` for linear modes, or the
`init -> init_loop -> start -> loop -> stop` callbacks for iterative modes.
Configure `startAfterTicks`, `stopAfterTicks`, `beforeTick` and `onStop`, then
call `run()` and assert its `COMPLETED`, `STOP_REQUESTED` or `TIMED_OUT` outcome.
Script gamepad fields in `beforeTick`; SDK button aliases and press/release
queues are updated before user code runs.
Assert against `runner.opMode()` rather than the original template: the SDK
runtime spy copies instance fields. For linear modes, register explicit cleanup
with `onStop` because the SDK's final `stop()` is a no-op; user `finally` blocks
still execute. The tick limit bounds cooperative SDK calls, not arbitrary Java
loops. Test-scoped globals assume ordinary, non-parallel JUnit execution.

The robot constructor accepts a `Clock` and follower factory for simulation.
The match entries expose a protected `createRobot()` hook so test subclasses can
inject that robot without replacing the OpMode's initialization, control,
planning or cleanup. Normal constructors still use the system clock and the
configured Pinpoint/Mecanum follower.

Examples to copy and adapt:

| Example | What it demonstrates |
| --- | --- |
| `fakeHw/FakeMotorTest.java` | SDK motor/controller behavior, encoder integration and device faults |
| `mechanisms/FlywheelSimulationTest.java` | Real closed-loop flywheel control, voltage sag, load recovery and readiness |
| `mechanisms/ShootOnMoveSimulationTest.java` | Real moving-shot solutions and heading/velocity compensation |
| `testing/RobotTestRigTest.java` | Shared battery reaches real consumers; SOTM aiming retains real path translation |
| `testing/OpModeRunnerTest.java` | Iterative/linear lifecycle, gamepad input, timeout outcomes and failure cleanup |
| `opmodes/auto/MatchOpModeSimulationTest.java` | Real match OpModes with fake hardware and measured-wheel-speed odometry |

Paths in this table are relative to
`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/`.

The SOTM regression exposed and fixes a follower allocation bug: the live heading
override now reaches the path algorithm **before** it allocates translation/turn
power. Replacing turn power afterward allowed the old heading error to consume
the power budget and stall translation. Releasing the override restores the
original interpolation without modifying the reusable path or restarting it.

**Simulation is not robot qualification.** Motor plants and planar odometry are
configurable idealized models, not measured Chimera dynamics. No-camera tests
exercise vision timeout behavior, not image processing, tag accuracy or pipeline
latency. A timed feed is an attempted batch, not proof of a score. Raw
`System.nanoTime`, `Thread.sleep`, native drivers and arbitrary busy loops are
outside deterministic stepping; use the provided clock/lifecycle seams.

Before deploying control or autonomous changes, use the existing Driver Station
flywheel/aim/follower development OpModes to measure both wheel velocities,
battery-sag response and stopping behavior, then validate localization, SOTM
accuracy and autonomous paths/clearances on the configured robot. Restore match
settings after tuning; desktop results do not establish safe physical clearances
or tuned gains.
