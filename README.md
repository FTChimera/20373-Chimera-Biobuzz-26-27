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

- **Dijkstra's shortest path for parking.** The endgame park route is planned on a visibility graph: the field is reduced to waypoints (start, goal, a 3x3 lattice of fixed points, and obstacle corners) and [Dijkstra's algorithm](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/game/DijkstraPathfinder.java) finds the shortest path around forbidden space — the hive bases and the flowers — using the [JTS Topology Suite](https://github.com/locationtech/jts) for the geometry.
- **Kalman-filter localization.** [`FusionLocalizer`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/FusionLocalizer.java) fuses dead reckoning with measurements through a Kalman filter (covariance tracking, history buffer), and [`util/control/filters/`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/filters) holds a hand-written extended Kalman filter for nonlinear models.
- **PIDSVA flywheel control, tuned automatically.** The two-motor shooter runs a full velocity controller — PID plus feedforward (kS/kV/kA) with voltage compensation — and its constants come from a custom PIDSVA autotuner (relay test + Ziegler-Nichols + least-squares fit), built on [Aarav Dhawan's FtcAutoTune](https://github.com/aaravdhawan25/FtcAutoTune).
- **Command-based autonomous.** Autos are composed from [Pedro Ivy](https://github.com/Pedro-Pathing/PedroPathing) commands (`PlanBuilder` + `PathPlanner`), so each routine is a chain of small reusable steps: drive a path, intake, line up on the hive, shoot, park.

## Libraries

- [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing) 3.0.2 (+ tuning 1.0.1) — path following and localization
- [Pedro Ivy](https://github.com/Pedro-Pathing/PedroPathing) 1.1.1 — command framework for autonomous
- [FtcAutoTune](https://github.com/aaravdhawan25/FtcAutoTune) 1.4.3 — PID/PIDF autotuning (relay + Ziegler-Nichols)
- [BehaviorArchitectures](https://github.com/shlok-k720/BehaviorArchitectures) 0.2.0 — our own behavior-tree, command, and finite-state-machine library
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
planners/     Autonomous planning: PlanBuilder, PathPlanner, field poses (Common.kt)
robotDev/     Development and tuning tools: AutoTuner, subsystem tests
util/         Shared building blocks, grouped by job:
  autotune/   Relay tuner, Ziegler-Nichols, S/V/A fit
  control/    PID/PIDF controllers, filters (incl. extended Kalman filter)
  game/       Field model, hive state, Dijkstra pathfinder
  hardware/   SmartFlywheel, cached motors, gamepad helpers
  shotcalc/   Shot tables and the ideal shot model (per motor)
  ...plus math, poses, interpolation, localization
```
