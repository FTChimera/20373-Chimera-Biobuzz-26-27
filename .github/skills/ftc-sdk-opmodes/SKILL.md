---
name: ftc-sdk-opmodes
description: Use this skill to create or edit FTC SDK op modes in TeamCode.
---

# FTC SDK Op Modes

Use the current concrete match entries rather than creating a parallel architecture.

## Registered competition entries

- `tele/TeleOp.java` is the teleoperated implementation.
- It is a `LinearOpMode`.
- It has the competition teleop annotation.
- Its nested `Red_Tele` and `Blue_Tele` classes provide alliance choices.
- `autos/Garden8.java` is the autonomous implementation.
- It is an iterative `OpMode`.
- Its nested `redalliance` and `bluealliance` classes are the autonomous choices.
- Both autonomous choices preselect the configured teleop name.

## Choosing an op mode shape

- Use `LinearOpMode` when sequential initialization and an active-loop structure fit.
- Use iterative `OpMode` when scheduler execution belongs in `loop()`.
- Do not mix `waitForStart()` semantics into an iterative mode.
- Do not block an iterative callback waiting for a command or sensor.
- Keep driver-station annotations on concrete, source-backed entries.

## Teleop pattern

1. Create `DualGamepad` from the SDK gamepads.
2. Create `State(RobotMode.TELEOP)`.
3. Create `Robot` through `Constants.createRobot(hardwareMap, RobotMode.TELEOP)`.
4. Resolve alliance and starting pose from `Blackboard`.
5. Configure drive curve and start teleop drive.
6. Set `GpBindings` on the state.
7. Call `waitForStart()`.
8. Start the robot.
9. In the active loop update gamepads and call `robot.periodic(state)`.
10. Stop the robot after the loop.

## Autonomous pattern

1. Reset `Scheduler` in `init()`.
2. Build the robot and its `PlanBuilder` route.
3. Start the robot in `start()`.
4. Execute the scheduler every `loop()`.
5. Apply the autonomous time limit and park behavior.
6. Stop once, then reset the scheduler in `stop()`.
7. Preserve the pose handoff through the aggregate robot lifecycle.

## Alliance and handoff

- Put selected alliance values in `Blackboard`.
- `TeleOp` prefers a Blackboard pose when one exists.
- It otherwise uses the selected alliance default pose.
- `Robot.saveEndVals()` saves autonomous pose and alliance only after periodic state is available.
- Do not manually duplicate handoff logic in each entry unless behavior intentionally differs.

## Mechanism use

- Create mechanisms through the aggregate robot, not directly from an op mode.
- Use `requireDrive()` only when drive is profile-mandatory.
- Use optional getters for profile-dependent equipment.
- Each aggregate periodic call supplies `State` to configured mechanisms.
- Start and stop vision through the robot API.

## New entries

- Add an annotation only when the class is meant to appear on the Driver Station.
- Give the annotation a stable name and competition group.
- Decide explicitly whether the mode should preserve pose handoff.
- Reuse `PlanBuilder`, `Common`, `Robot`, and `DualGamepad` where their existing behavior fits.
- Do not copy stale base-class patterns or deleted test examples.
- Keep exploratory code in existing test or learning areas rather than registering it as competition code.

## Verification

- Compile TeamCode.
- Confirm annotated names are unique in source.
- Confirm the active profile supplies required mechanisms.
- Confirm teleop starts after autonomous with the expected pose.
- Confirm stop reliably clears outputs and scheduler state.

## Documentation requirements

- Describe an entry by its actual package and class name.
- Describe only annotations that exist in current Java source.
- State whether the entry is linear or iterative.
- State how it creates the aggregate robot and state mode.
- State whether it participates in Blackboard pose handoff.
- Keep test-only entry points separate from competition entry documentation.
- Remove references to deleted base classes rather than preserving path lists.
- Recheck entry documentation after moving or renaming an annotated class.

## Change boundaries

- Keep annotations, lifecycle, and Driver Station labels in the entry class.
- Keep hardware construction in `Constants` and profile factories.
- Keep route composition in planner classes and mechanism work in the aggregate.
- Verify these ownership boundaries before adding a new competition selection.
