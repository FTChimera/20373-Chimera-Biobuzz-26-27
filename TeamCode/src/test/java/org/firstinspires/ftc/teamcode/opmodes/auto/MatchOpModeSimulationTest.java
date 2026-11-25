package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.ICommand;
import com.rmsftc.cmdlib.cmd.groups.SequentialCommandGroup;
import com.rmsftc.cmdlib.cmd.decorators.WrapperCommand;
import com.rmsftc.cmdlib.scheduler.SchedulerEventListener;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.firstinspires.ftc.teamcode.testing.OpModeRunner;
import org.firstinspires.ftc.teamcode.testing.RecordingTelemetry;
import org.firstinspires.ftc.teamcode.testing.RobotTestRig;
import org.junit.Rule;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

/** Full runOpMode examples: only createRobot is overridden; no plan or mechanism is replaced. */
public class MatchOpModeSimulationTest {
    private static final long STEP_NS = 20_000_000;
    private static final double STEP_SEC = STEP_NS / 1e9;
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    public static final class TeleOpExample extends MainTeleOp {
        private final RobotTestRig rig;
        public TeleOpExample(RobotTestRig rig) { this.rig = rig; }
        @Override protected Robot createRobot() {
            return rig.createRobot(telemetry, OpModeStorage.allianceColor.get(),
                    RobotMode.TELEOP, OpModeStorage.pose.get());
        }
    }
    public static final class GardenExample extends Garden8 {
        private final RobotTestRig rig;
        public GardenExample(RobotTestRig rig) { this.rig = rig; }
        @Override protected Robot createRobot() {
            return rig.createRobot(telemetry, OpModeStorage.allianceColor.get(),
                    RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        }
    }
    public static final class LoadingExample extends Loading8 {
        private final RobotTestRig rig;
        public LoadingExample(RobotTestRig rig) { this.rig = rig; }
        @Override protected Robot createRobot() {
            return rig.createRobot(telemetry, OpModeStorage.allianceColor.get(),
                    RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        }
    }

    private static final class PlanEvents implements SchedulerEventListener {
        final List<ICommand> scheduled = new ArrayList<>();
        final List<ICommand> finished = new ArrayList<>();
        final List<ICommand> cancelled = new ArrayList<>();
        @Override public void onScheduled(ICommand command) { scheduled.add(command); }
        @Override public void onFinished(ICommand command) { finished.add(command); }
        @Override public void onInterrupted(ICommand command) { cancelled.add(command); }
        @Override public void onCancelled(ICommand command) { cancelled.add(command); }
    }

    private void configureMatch() {
        OpModeStorage.allianceColor.set(AllianceColor.BLUE);
        OpModeStorage.matchMode.set(MatchMode.QUALIFICATIONS);
        OpModeStorage.initialized.set(false);
        Shooter.useSotm.set(false);
    }

    private <T extends OpMode> OpModeRunner<T> runner(T mode, RobotTestRig rig,
                                                    RecordingTelemetry telemetry) {
        return new OpModeRunner<>(mode, rig.hardwareMap, telemetry, simulation.clock,
                STEP_NS, 1800).startAfterTicks(5).beforeTick(r -> rig.advance(STEP_SEC));
    }

    @Test public void teleOpRunsRealInputsAndStopsEveryMotorOnDriverStop() throws Exception {
        configureMatch();
        OpModeStorage.pose.set(new Pose(36, 36, 0));
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            RecordingTelemetry telemetry = new RecordingTelemetry();
            OpModeRunner<TeleOpExample> runner = runner(new TeleOpExample(rig), rig, telemetry)
                    .stopAfterTicks(400).beforeTick(r -> {
                        rig.advance(STEP_SEC);
                        int tick = r.ticks();
                        r.opMode().gamepad1.left_stick_y = tick < 100 ? -0.5f : 0;
                        r.opMode().gamepad1.left_stick_x = tick >= 100 && tick < 150 ? -0.3f : 0;
                        r.opMode().gamepad1.right_stick_x = tick >= 150 && tick < 180 ? -0.2f : 0;
                        r.opMode().gamepad2.start = tick == 20;
                        r.opMode().gamepad2.a = tick == 30;
                        r.opMode().gamepad2.right_trigger = tick >= 50 && tick < 100 ? 1 : 0;
                        r.opMode().gamepad2.b = tick >= 200 && tick < 250;
                        r.opMode().gamepad2.left_stick_button = tick >= 270 && tick < 300;
                        r.opMode().gamepad2.back = tick >= 320 && tick < 350;
                    });
            OpModeRunner.Result result = runner.run();
            rig.observe();
            assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, result.outcome);
            assertEquals(400, result.ticks);
            assertTrue("Drive sticks must move measured odometry", rig.distanceTravelled() > 20);
            assertTrue("Intake toggles/deployment must reach hardware", rig.deployer.getPositionHistory().size() >= 3);
            assertTrue("Panic feed must reach the real transfer", rig.transferTransitions().stream().anyMatch(p -> p > 0));
            assertTrue("Eject must reverse the transfer", rig.transferTransitions().stream().anyMatch(p -> p < 0));
            assertTrue("Real shooter controller must write power", rig.mainShooter.getPowerHistory().stream().anyMatch(p -> p > 0));
            assertTrue("Reverse input must reach flywheel", rig.mainShooter.getPowerHistory().stream().anyMatch(p -> p < 0));
            assertFalse("TeleOp endMatch clears initialized", OpModeStorage.initialized.get());
            assertStopped(rig);
            assertFalse(telemetry.updates().isEmpty());
        }
    }

    @Test public void gardenRunsPlannedLegsFeedsAndParksUntilMatchDeadline() throws Exception {
        runAutonomous(false, false);
    }

    @Test public void loadingRunsPlannedLegsFeedsAndParksUntilMatchDeadline() throws Exception {
        runAutonomous(true, false);
    }

    @Test public void stalledGardenDoesNotInventPathOrShotSuccessAndStillStopsAtDeadline() throws Exception {
        runAutonomous(false, true);
    }

    private void runAutonomous(boolean loading, boolean stalled) throws Exception {
        configureMatch();
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            rig.setDriveStalled(stalled);
            RecordingTelemetry telemetry = new RecordingTelemetry();
            OpMode mode = loading ? new LoadingExample(rig) : new GardenExample(rig);
            OpModeRunner<OpMode> runner = runner(mode, rig, telemetry);
            ArchitectureManager manager = (ArchitectureManager) readField(
                    ArchitecturalOpMode.class, runner.opMode(), "manager");
            PlanEvents events = new PlanEvents();
            manager.scheduler().addEventListener(events);
            OpModeRunner.Result result = runner.run();
            rig.observe();
            assertEquals("The OpMode, not the safety tick cap, must end the match",
                    OpModeRunner.Outcome.STOP_REQUESTED, result.outcome);
            assertTrue("Run the full autonomous match", rig.robot.getOpModeTimeSeconds() >= 30);
            assertTrue("Bounded deadline overshoot", rig.robot.getOpModeTimeSeconds() < 30.1);
            assertTrue(OpModeStorage.initialized.get());
            assertStopped(rig);
            assertStoredPose(rig);
            Command plan;
            if (loading) {
                Loading8 actual = (Loading8) runner.opMode();
                plan = actual.plan;
                assertTrue(actual.isStopped);
                assertTrue("Parking must be attempted", actual.startParked);
            } else {
                Garden8 actual = (Garden8) runner.opMode();
                plan = actual.plan;
                assertTrue(actual.isStopped);
                assertTrue("Parking must be attempted", actual.startParked);
            }
            assertTrue("The actual match plan must be scheduled", events.scheduled.contains(plan));
            assertFalse("Deadline/parking must unschedule the real plan", manager.scheduler().isScheduled(plan));
            assertTrue("Shutdown must leave no commands running", manager.scheduler().runningCommands().isEmpty());
            List<ShooterCommands.ShotCommand> shots = new ArrayList<>();
            collectShots(plan, shots);
            if (stalled) {
                assertEquals(0, rig.distanceTravelled(), 1e-12);
                assertTrue("Stall must not manufacture completed path progress",
                        rig.pathProgress().values().stream().noneMatch(p -> p >= 1));
                assertTrue("Unfinished preload leg must not feed",
                        rig.transferTransitions().stream().noneMatch(p -> p > 0));
                assertTrue("A stalled preload leg must not claim an attempted shot",
                        shots.stream().noneMatch(ShooterCommands.ShotCommand::attempted));
                assertTrue("Parking must cancel the blocked real plan", events.cancelled.contains(plan));
                assertFalse("Stall must not report natural plan completion", events.finished.contains(plan));
            } else {
                assertTrue("Preloads, intake, return and park must create distinct path runs: "
                        + rig.pathProgress(), rig.pathProgress().size() >= 4);
                assertTrue("Real Foresight must advance paths, not merely start them",
                        rig.pathProgress().values().stream().anyMatch(p -> p > 0.8));
                assertTrue("Measured drive motion must cover planned legs", rig.distanceTravelled() > 40);
                List<Double> feeds = completedFeedDurations(rig);
                assertTrue("Two real attempted batches must feed then stop: " + feeds, feeds.size() >= 2);
                assertEquals("Both planned batch commands must exist", 2, shots.size());
                assertTrue("Both real planner legs must finish before parking", events.finished.contains(plan));
                for (ShooterCommands.ShotCommand shot : shots) {
                    assertTrue("Real shot lifecycle must classify a timed feed as attempted: "
                            + shot.outcome(), shot.attempted());
                    assertTrue(shot.outcome() == ShooterCommands.ShotOutcome.ATTEMPTED
                            || shot.outcome() == ShooterCommands.ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT);
                }
                for (double duration : feeds) {
                    assertTrue("A completed feed is an attempt, not proof of a score",
                            duration >= org.firstinspires.ftc.teamcode.planners.Constants.Timings.shootingTimeMs / 1000.0);
                }
                assertTrue("Stub vision should report blind lineup, not invent a camera frame",
                        telemetry.updates().stream().flatMap(u -> u.entries.stream())
                                .anyMatch(e -> e.caption.equals("Hive lineup timeout")));
            }
        }
    }

    // Vendor groups and PathPlanner intentionally expose no child accessor. Read only these
    // known composition fields; never change them or execute a command outside its real plan.
    private static void collectShots(Object command, List<ShooterCommands.ShotCommand> shots)
            throws ReflectiveOperationException {
        if (command instanceof ShooterCommands.ShotCommand) {
            shots.add((ShooterCommands.ShotCommand) command);
        } else if (command instanceof SequentialCommandGroup) {
            for (Object child : (List<?>) readField(SequentialCommandGroup.class, command, "children")) {
                collectShots(child, shots);
            }
        } else if (command instanceof PathPlanner) {
            collectShots(readField(PathPlanner.class, command, "inner"), shots);
        } else if (command instanceof WrapperCommand) {
            collectShots(readField(WrapperCommand.class, command, "wrapped"), shots);
        }
    }

    private static Object readField(Class<?> owner, Object instance, String name)
            throws ReflectiveOperationException {
        java.lang.reflect.Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(instance);
    }

    private static List<Double> completedFeedDurations(RobotTestRig rig) {
        List<Double> durations = new ArrayList<>();
        boolean feeding = false;
        double start = 0;
        for (RobotTestRig.TransferTransition transition : rig.timedTransferTransitions()) {
            if (transition.power > 0 && !feeding) {
                start = transition.seconds;
                feeding = true;
            } else if (transition.power <= 0 && feeding) {
                durations.add(transition.seconds - start);
                feeding = false;
            }
        }
        return durations;
    }

    private static void assertStopped(RobotTestRig rig) {
        for (FakeDcMotorEx motor : rig.motors) assertEquals("Shutdown motor power", 0, motor.getPower(), 0);
        assertEquals(Follower.Mode.IDLE, rig.robot.follower.mode());
        assertFalse(rig.robot.smartIntakeControl);
        assertFalse(rig.robot.smartShooterControl);
        assertFalse(rig.robot.intake.isDeployed());
    }

    private static void assertStoredPose(RobotTestRig rig) {
        Pose expected = rig.robot.getPoseCompensatedForVelocity();
        Pose actual = OpModeStorage.pose.get();
        assertEquals(expected.x(), actual.x(), 1e-9);
        assertEquals(expected.y(), actual.y(), 1e-9);
        assertEquals(expected.heading(), actual.heading(), 1e-9);
    }
}
