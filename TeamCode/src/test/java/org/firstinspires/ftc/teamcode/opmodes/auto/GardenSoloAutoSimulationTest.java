package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Velocity;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands.ShotOutcome;
import org.firstinspires.ftc.teamcode.planners.tree.TreeShooterCommands;
import org.firstinspires.ftc.teamcode.planners.tree.TreeState;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.firstinspires.ftc.teamcode.testing.OpModeRunner;
import org.firstinspires.ftc.teamcode.testing.RecordingTelemetry;
import org.firstinspires.ftc.teamcode.testing.RobotTestRig;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.shotcalc.IdealShotCalc;
import org.junit.Rule;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.*;

/** Real tree, solver, controllers and runOpMode; only the electrical/runtime boundaries are fake. */
public class GardenSoloAutoSimulationTest {
    private static final long STEP_NS = 20_000_000;
    private static final double STEP_SEC = STEP_NS / 1e9;
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    public static final class GardenSoloExample extends GardenSoloAuto {
        private final RobotTestRig rig;
        public GardenSoloExample(RobotTestRig rig) { this.rig = rig; }
        @Override protected Robot createRobot() {
            return rig.createRobot(telemetry, OpModeStorage.allianceColor.get(),
                    RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        }
    }

    @Test public void bluePreloadsSpinAndFeedAtGardenThenBeginGardenToLoading() throws Exception {
        runPreloads(AllianceColor.BLUE);
    }

    @Test public void redPreloadsSpinAndFeedAtGardenThenBeginGardenToLoading() throws Exception {
        runPreloads(AllianceColor.RED);
    }

    private void configure(AllianceColor alliance) {
        OpModeStorage.allianceColor.set(alliance);
        OpModeStorage.matchMode.set(MatchMode.QUALIFICATIONS);
        OpModeStorage.initialized.set(false);
        Shooter.useSotm.set(false);
    }

    private void runPreloads(AllianceColor alliance) throws Exception {
        configure(alliance);
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            RecordingTelemetry telemetry = new RecordingTelemetry();
            PreloadObservations observations = new PreloadObservations();
            OpModeRunner<GardenSoloExample> runner = new OpModeRunner<>(
                    new GardenSoloExample(rig), rig.hardwareMap, telemetry,
                    simulation.clock, STEP_NS, 1000).startAfterTicks(5)
                    .beforeTick(r -> {
                        rig.advance(STEP_SEC);
                        if (rig.robot == null || !r.started()) return;
                        TreeState state = treeState(r.opMode());
                        observations.observe(rig, telemetry, state);
                        if (state.attemptedBatches() == 1
                                && rig.intake.getPower() > 0
                                && rig.robot.follower.following()
                                && rig.robot.getPose().distance(TreeState.poseFor(Position.GARDEN_SHOOT)) > 2) {
                            observations.nextLegStarted = true;
                            r.requestStop();
                        }
                    });
            OpModeRunner.Result result = runner.run();
            rig.observe();
            TreeState state = treeState(runner.opMode());
            assertEquals("Driver STOP, not the safety tick cap", OpModeRunner.Outcome.STOP_REQUESTED, result.outcome);
            assertTrue("Actual GardenToLoading intake route must begin", observations.nextLegStarted);
            assertTrue(rig.robot.shooter.shotCalculator instanceof IdealShotCalc);
            assertTrue("Both wheels must have real power and measured velocity before feeding: "
                    + observations.spinningPhases, observations.spinningPhases.contains("APPROACH"));
            assertTrue(observations.spinningPhases.contains("SETTLE"));
            assertTrue(observations.spinningPhases.contains("VISION"));
            assertFalse("No preload side switch", observations.phases.contains("SWITCH"));
            assertTrue("Real transfer must feed", Double.isFinite(observations.firstFeedSeconds));
            assertTrue("First feed must remain at garden: " + observations.firstFeedPose,
                    observations.firstFeedPose.distance(TreeState.poseFor(Position.GARDEN_SHOOT))
                            <= Constants.Tree.Shoot.POSITION_TOLERANCE_IN);
            assertTrue("First feed must not be at loading",
                    observations.firstFeedPose.distance(TreeState.poseFor(Position.LOADING_SHOOT)) > 20);
            assertTrue("No-tag lineup gets its full bounded wait",
                    observations.firstFeedSeconds - observations.firstVisionSeconds
                            >= Constants.Timings.hiveLineupTimeoutTime - STEP_SEC);
            assertEquals(1, state.attemptedBatches());
            assertEquals(Position.GARDEN_SHOOT, state.position());
            assertEquals(TreeState.Selection.GARDEN_TO_LOADING, state.selectLeg());
            assertEquals(Field.HiveState.TIPPED, state.predictedHiveState());
            assertTrue("Measured settling, not manufactured readiness", state.lastShotSettled());
            assertFalse("Vision.stub must not invent a tag", state.lastShotTagConfirmed());
            assertTrue(state.predictionUncertain());
            assertTrue(state.intakesDone().isEmpty());
            assertTrue(state.lastShotOutcome() == ShotOutcome.ATTEMPTED
                    || state.lastShotOutcome() == ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT);
            assertCompletedFeed(rig);
            assertTrue(runner.opMode().isStopped);
            assertStopped(rig);
            System.out.println(alliance + " garden preload before feed: " + observations.visionDebug
                    + "; first feed=" + observations.firstFeedSeconds + "s pose=" + observations.firstFeedPose);
        }
    }

    private static final class PreloadObservations {
        final Set<String> phases = new HashSet<>();
        final Set<String> spinningPhases = new HashSet<>();
        double firstVisionSeconds = Double.NaN;
        double firstFeedSeconds = Double.NaN;
        Pose firstFeedPose;
        String visionDebug;
        boolean nextLegStarted;

        void observe(RobotTestRig rig, RecordingTelemetry telemetry, TreeState state) {
            if (state.attemptedBatches() != 0) return;
            String phase = telemetry.updates().isEmpty() ? null
                    : telemetry.latest().values().get("Tree shooting phase");
            if (phase == null) phase = "APPROACH";
            phases.add(phase);
            if (phase.equals("VISION") && !Double.isFinite(firstVisionSeconds)) {
                firstVisionSeconds = rig.clock.nowSeconds();
            }
            if (!Double.isFinite(firstFeedSeconds) && rig.transfer.getPower() > 0) {
                firstFeedSeconds = rig.clock.nowSeconds();
                firstFeedPose = rig.robot.getPose();
            }
            if (!Double.isFinite(firstFeedSeconds)
                    && rig.mainShooter.getPower() > 0 && rig.counterShooter.getPower() > 0
                    && rig.mainShooter.getVelocity() > 0 && rig.counterShooter.getVelocity() > 0
                    && (phase.equals("APPROACH") || rig.robot.getPose().distance(
                            TreeState.poseFor(Position.GARDEN_SHOOT)) <= Constants.Tree.Shoot.POSITION_TOLERANCE_IN)) {
                spinningPhases.add(phase);
                if (phase.equals("VISION")) visionDebug = rig.robot.shooter.getDebugString();
            }
        }
    }

    @Test public void normalNoTagShotStillSwitchesAndFeedsOnlyAfterMeasuredLoadingArrival() {
        configure(AllianceColor.BLUE);
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            RecordingTelemetry telemetry = new RecordingTelemetry();
            Robot robot = rig.createRobot(telemetry, AllianceColor.BLUE, RobotMode.AUTONOMOUS,
                    Constants.ShootPoses.garden);
            robot.start();
            robot.setSmartShooterControl(true);
            TreeState state = new TreeState();
            state.initialize();
            TreeShooterCommands.ShootCommand shot = TreeShooterCommands.shoot(
                    robot, state, Position.GARDEN_SHOOT, TreeState.IntakeArea.WEST_FLOWER);
            Status status = Status.RUNNING;
            Pose firstFeedPose = null;
            boolean switchObserved = false;
            for (int tick = 0; tick < 1000 && status == Status.RUNNING; tick++) {
                simulation.clock.add(STEP_NS);
                rig.advance(STEP_SEC);
                robot.update();
                status = shot.execute();
                rig.observe();
                telemetry.update();
                switchObserved |= "SWITCH".equals(telemetry.latest().values().get("Tree shooting phase"));
                if (firstFeedPose == null && rig.transfer.getPower() > 0) {
                    firstFeedPose = robot.getPose();
                    assertTrue("Actual wheels must spin, not fabricated readiness",
                            rig.mainShooter.getVelocity() > 0 && rig.counterShooter.getVelocity() > 0);
                }
            }
            rig.observe();
            assertNotEquals("Normal shot must finish boundedly", Status.RUNNING, status);
            assertTrue("No-tag switch remains intentional on normal shots", switchObserved);
            assertTrue(shot.switchedSides());
            assertNotNull("Feed must reach actual transfer hardware", firstFeedPose);
            assertTrue("Never feed before measured switched-side arrival: " + firstFeedPose,
                    firstFeedPose.distance(TreeState.poseFor(Position.LOADING_SHOOT))
                            <= Constants.Tree.Shoot.POSITION_TOLERANCE_IN);
            assertEquals(Position.LOADING_SHOOT, state.position());
            assertEquals(1, state.attemptedBatches());
            assertTrue(state.intakesDone().contains(TreeState.IntakeArea.WEST_FLOWER));
            assertFalse(state.controlPathAborted());
            assertFalse(shot.tagSeen());
            assertTrue("Real corridor traversal must move measured odometry", rig.distanceTravelled() > 40);
            assertCompletedFeed(rig);
            robot.stopEverything();
            assertStopped(rig);
        }
    }

    @Test public void infeasibleIdealTargetAtGardenNeverFeedsOrSwitchesSides() {
        configure(AllianceColor.BLUE);
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            RecordingTelemetry telemetry = new RecordingTelemetry();
            Robot robot = rig.createRobot(telemetry, AllianceColor.BLUE, RobotMode.AUTONOMOUS,
                    Constants.ShootPoses.garden);
            robot.start();
            // Exercise the existing solveAndSetVelocity seam with an infeasible zero-distance
            // context, while the real follower holds garden. Do not replace solver/readiness
            // or write measured motor signals; disable automatic context replacement only.
            robot.setSmartShooterControl(false);
            robot.shooter.solveAndSetVelocity(Field.HiveState.UNTIPPED.goalPose(AllianceColor.BLUE),
                    Velocity.zero(), Field.HiveState.UNTIPPED, STEP_SEC);
            assertTrue(robot.shooter.shotCalculator instanceof IdealShotCalc);
            assertFalse(robot.shooter.hasNonzeroVelocityTarget());
            TreeState state = new TreeState();
            state.initialize();
            TreeShooterCommands.ShootCommand shot = TreeShooterCommands.shootPreloads(robot, state);
            Status status = Status.RUNNING;
            for (int tick = 0; tick < 500 && status == Status.RUNNING; tick++) {
                simulation.clock.add(STEP_NS);
                rig.advance(STEP_SEC);
                robot.update();
                status = shot.execute();
                telemetry.update();
                assertEquals(0, rig.transfer.getPower(), 0);
                assertEquals(0, rig.mainShooter.getPower(), 0);
                assertEquals(0, rig.counterShooter.getPower(), 0);
                assertEquals(0, rig.mainShooter.getVelocity(), 0);
                assertEquals(0, rig.counterShooter.getVelocity(), 0);
                assertFalse(robot.shooter.upToSpeed());
                assertFalse(shot.switchedSides());
                assertEquals("No side-switch path may start", 0, robot.follower.pathRunId());
            }
            assertNotEquals("Bounded skip must finish", Status.RUNNING, status);
            assertEquals(ShotOutcome.SKIPPED, shot.outcome());
            assertTrue(shot.poseSettled());
            assertFalse(shot.tagSeen());
            assertEquals(0, state.attemptedBatches());
            assertEquals(ShotOutcome.SKIPPED, state.lastShotOutcome());
            assertEquals(Position.GARDEN_SHOOT, state.position());
            assertEquals(Field.HiveState.UNTIPPED, state.predictedHiveState());
            assertFalse(state.controlPathAborted());
            assertEquals(0, rig.distanceTravelled(), 1e-9);
            assertTrue(rig.transferTransitions().isEmpty());
            robot.stopEverything();
            assertStopped(rig);
        }
    }

    private static TreeState treeState(GardenSoloAuto mode) {
        try {
            java.lang.reflect.Field field = GardenSoloAuto.class.getDeclaredField("state");
            field.setAccessible(true);
            return (TreeState) field.get(mode);
        } catch (ReflectiveOperationException exception) {
            throw new AssertionError(exception);
        }
    }

    private static void assertCompletedFeed(RobotTestRig rig) {
        double start = Double.NaN;
        for (RobotTestRig.TransferTransition transition : rig.timedTransferTransitions()) {
            if (transition.power > 0 && !Double.isFinite(start)) {
                start = transition.seconds;
            } else if (transition.power <= 0 && Double.isFinite(start)) {
                assertTrue("Full timed batch attempt, not merely a feed pulse",
                        transition.seconds - start >= Constants.Timings.shootingTimeMs / 1000.0);
                return;
            }
        }
        fail("No completed timed feed");
    }

    private static void assertStopped(RobotTestRig rig) {
        for (FakeDcMotorEx motor : rig.motors) assertEquals("Driver STOP motor power", 0, motor.getPower(), 0);
        assertEquals(Follower.Mode.IDLE, rig.robot.follower.mode());
        assertFalse(rig.robot.smartShooterControl);
        assertFalse(rig.robot.smartIntakeControl);
        assertFalse(rig.robot.intake.isDeployed());
    }
}
