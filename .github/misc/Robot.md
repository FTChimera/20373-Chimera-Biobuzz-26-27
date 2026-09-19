# Robot Design Plan

This is the current BioBuzz design direction for `FirstRobot`, not a final hardware
specification. Measurements, motor selection, sensing, and calibration need testing.

## System summary

The planned material path has three systems:
1. a front intake;
2. a hooded flywheel launcher; and
3. a first-in, first-out transfer.

The launcher has no turret. Auto-alignment will rotate the complete chassis
toward the hive using the current hive state.

## Software-addressable hardware
This section records hardware that will require TeamCode configuration,
parameters, telemetry, or control logic. It intentionally excludes purely
mechanical connections and packaging.
| Planned hardware | Software implication | Current status |
| --- | --- | --- |
| Intake motor group | Motor count determines whether `Intake` remains single-motor or needs a synchronized multi-motor interface, hardware names, directions, and power control. | Open: one or two motors. |
| Intake lift actuator | Requires a chosen actuator, hardware name, limits, and control interface before lift behavior can be coded. | Not selected. |
| Flywheel motor | Requires `Shooter` parameters for hardware name, direction, velocity conversion, and controller tuning. | One 6000 RPM motor planned; mechanism is scaffolded. |
| Ball-identification sensor | Requires a selected SDK/wrapper integration and a reliable pollen-versus-nectar result for launch selection. | Sensor type and placement open. |
| Hood servo | Requires a position interface and calibrated targets only if active hood adjustment is adopted. | Conditional. |
| Transfer motor | Requires a hardware name, direction, power behavior, and transfer mechanism interface. | One motor planned; no transfer mechanism exists yet. |
| Hive-state auto-align | Requires meaningful `HiveState` / `FieldUpdates` data plus a chassis rotation command. | Planned; current game-state helpers are scaffolds. |

The table is a software planning contract, not confirmation that FirstRobot is
already wired for these devices. Implement hardware-map names and parameters
only after the associated choices are settled.
## Intake
The robot will have one front intake system. Its main intake is farther inside
the robot and approximately 14 inches wide.
That width will not reliably capture game elements entering at the front
corners. A horizontal roller at each front side will guide corner entries into
the main intake.
The two side rollers and main intake will be mechanically connected. The
connected intake assembly is intended to use one or two 1150 RPM motors total.
The intake will lift so it can collect both pollen and nectar. No sorting
system is planned because the intended flow does not require it.

### Intake hardware budget
- Main intake and side rollers: one or two 1150 RPM motors total.
- Intake lift: actuator and packaging are not yet specified.

### Intake open decision
Decide whether the connected rollers and main intake use one motor or two.
Make that decision from the mechanical layout, load estimate, roller ratio, and
prototype testing.
## Launcher
The launcher is a single non-turreted hood launcher with one primary 6000 RPM
flywheel motor.
The hood module has a main hood connected to a semi-hood by a spring. The
semi-hood can compress when nectar is launched.
This concept is intended to launch both nectar and pollen. Because their
trajectories differ, the robot must identify the ball before firing.

Firing logic will adjust flywheel velocity for the detected ball type. Hood
angle may also need adjustment if velocity changes alone are insufficient.

### Launcher hardware budget
- One 6000 RPM flywheel motor.
- One ball-identification sensor.
- Possibly one hood-angle servo.
- Spring-coupled main hood and semi-hood hardware.

### Launcher open decisions
- Select the ball-identification sensor: evaluate color and distance sensing,
  brand, placement, and reliability with both game elements.
- Choose whether the launcher is on the intake side of the robot or opposite it.
- Determine from trajectory tests whether active hood-angle adjustment is needed.
- If hood adjustment is required, select and package a servo.

## Transfer
The transfer is a simple first-in, first-out path from intake to launcher. It
will use one motor and will not sort or buffer by game-element type.
## Current hardware estimate
- Intake system: one or two motors.
- Launcher flywheel: one motor.
- Transfer: one motor.
- Ball-identification sensor: required for the planned mixed-ball firing flow.
- Hood-angle servo: conditional on trajectory testing.

## Next design work

1. Draw packaging for the intake, side rollers, lift, transfer, and launcher.
2. Select the one-versus-two-motor intake design.
3. Prototype ball identification and record reliable sensor behavior.
4. Measure pollen and nectar launch trajectories.
5. Decide whether the hood needs active servo adjustment.
6. Choose launcher side and confirm chassis-based auto-alignment.
7. Move settled hardware names, parameters, and behavior into TeamCode owners.
