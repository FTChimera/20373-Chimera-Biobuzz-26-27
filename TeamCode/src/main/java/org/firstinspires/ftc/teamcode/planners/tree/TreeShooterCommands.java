package org.firstinspires.ftc.teamcode.planners.tree;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.PathPoses;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import com.rmsftc.utils.math.Pose;

import java.util.function.Consumer;
import com.rmsftc.utils.Clock;

/**
 * Bounded measured settling, then the plain tag rule on the latest camera
 * result: any alliance tag seen means shoot from this side; no tag within
 * the bounded wait means one measured side switch; still no tag after the
 * switch means a blind feed from the switched side. Missing
 * readiness/tag confirmation degrades a planned batch attempt, not a score.
 * A side switch is attempted at most once and must physically arrive before
 * feeding or advertising the destination as the next leg's start.
 */
public final class TreeShooterCommands {
    private static final double SHOOT_SETTLE_MS = 1500;
    private static final double SETTLE_DWELL_MS = 150;
    private static final double POSITION_TOLERANCE_IN = 2;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3);
    private static final double LINEAR_SPEED_TOLERANCE = 2;
    private static final double ANGULAR_SPEED_TOLERANCE = Math.toRadians(5);
    private static final double SWITCH_POSITION_TOLERANCE_IN = 1;
    private static final double SWITCH_TRAVERSE_TIMEOUT_MS = 6000;

    private TreeShooterCommands() {}

    /** A continuous measured dwell, never an instantaneous parametric/busy check. */
    static final class SettleWindow {
        private final long started;
        private final double timeoutMs;
        private long readySince;
        private boolean dwelling;

        SettleWindow(long now, double timeoutMs) {
            started = now;
            this.timeoutMs = timeoutMs;
        }

        boolean settled(boolean measuredReady, long now) {
            if (!measuredReady) {
                dwelling = false;
                return false;
            }
            if (!dwelling) {
                dwelling = true;
                readySince = now;
            }
            return elapsedMs(now, readySince) >= SETTLE_DWELL_MS;
        }

        boolean timedOut(long now) {
            return elapsedMs(now, started) >= timeoutMs;
        }
    }

    private static double elapsedMs(long now, long since) {
        return (now - since) / 1e6;
    }

    private static boolean settledAt(Robot robot, Pose pose, double positionTolerance) {
        return settledAt(robot.follower, pose, positionTolerance);
    }

    private static Pose pedroPose(Pose pose) {
        // FTCUtils Pose overrides accessors, but Pedro's distance/hold math reads base fields.
        return new Pose(pose.x(), pose.y(), pose.heading());
    }

    static Pose shootingHoldPose(Pose actual, Pose intended) {
        if (Math.hypot(actual.x() - intended.x(), actual.y() - intended.y()) > POSITION_TOLERANCE_IN) {
            return new Pose(actual.x(), actual.y(), intended.heading());
        }
        return pedroPose(intended);
    }

    static boolean canSwitchSides(boolean poseSettled, boolean alreadySwitched, double positionError) {
        return poseSettled && !alreadySwitched && positionError <= SWITCH_POSITION_TOLERANCE_IN;
    }

    private static boolean settledAt(Follower follower, Pose pose, double positionTolerance) {
        return follower.holding() && follower.hasControlUpdate()
                && follower.isPoseSettled(pedroPose(pose), positionTolerance,
                HEADING_TOLERANCE_RAD, LINEAR_SPEED_TOLERANCE, ANGULAR_SPEED_TOLERANCE);
    }

    public static ShootCommand shoot(Robot robot, TreeState state, Position intendedSide) {
        return shoot(robot, state, intendedSide, null);
    }

    public static ShootCommand shoot(Robot robot, TreeState state, Position intendedSide,
                                     TreeState.IntakeArea intakeArea) {
        return new ShootCommand(robot, state, intendedSide, intakeArea);
    }

    /** What the VISION phase does with the latest camera result. Pure. */
    enum VisionDecision { WAIT, SHOOT, SWITCH, BLIND_FEED }

    /**
     * The plain tag rule, as a pure decision: a tag in the latest result
     * shoots from this side at once; with no tag the phase waits out the
     * bounded lineup wait, then switches sides once when it is safe, and
     * otherwise feeds blind. {@code visionElapsedMs} is the time spent in
     * the VISION phase so far.
     */
    static VisionDecision decideVisionAction(boolean tagSeen, double visionElapsedMs,
                                             boolean poseSettled, boolean switched,
                                             double positionError) {
        if (tagSeen) return VisionDecision.SHOOT;
        if (visionElapsedMs < Timings.hiveLineupTimeoutTime * 1000.0) return VisionDecision.WAIT;
        if (canSwitchSides(poseSettled, switched, positionError)) return VisionDecision.SWITCH;
        return VisionDecision.BLIND_FEED;
    }

    public static final class ShootCommand extends Command {
        private enum Phase { SETTLE, VISION, SWITCH, FEED, DONE }

        private final Robot robot;
        private final TreeState state;
        private final Position intendedSide;
        private final TreeState.IntakeArea intakeArea;
        private Phase phase;
        private Position shotSide;
        private SettleWindow settle;
        private long visionStart;
        private boolean poseSettled;
        private boolean tagSeen;
        private boolean switched;
        private SideSwitchCommand sideSwitch;
        private ShooterCommands.ShotCommand feed;

        private ShootCommand(Robot robot, TreeState state, Position intendedSide, TreeState.IntakeArea intakeArea) {
            TreeState.otherShootPosition(intendedSide);
            this.robot = robot;
            this.state = state;
            this.intendedSide = intendedSide;
            this.intakeArea = intakeArea;
            addRequirements(robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        public ShooterCommands.ShotOutcome outcome() {
            return feed == null ? ShooterCommands.ShotOutcome.NOT_STARTED : feed.outcome();
        }

        public boolean poseSettled() { return poseSettled; }
        public boolean tagSeen() { return tagSeen; }
        public boolean switchedSides() { return switched; }

        @Override public void start() {
            shotSide = intendedSide;
            poseSettled = false;
            tagSeen = false;
            switched = false;
            feed = null;
            sideSwitch = null;
            if (state.controlPathAborted()) {
                phase = Phase.DONE;
                return;
            }
            Pose target = pedroPose(TreeState.poseFor(shotSide));
            Pose actual = Pose.fromPedro(robot.follower.localizer.pose());
            Pose hold = shootingHoldPose(actual, target);
            if (hold.x() != target.x() || hold.y() != target.y()) {
                // A timed-out curved approach must not become a straight-line shortcut.
                warn("Approach endpoint unverified; holding actual position for degraded planned attempt");
            }
            robot.follower.hold(hold);
            settle = new SettleWindow(Clock.SYSTEM.nowNanos(), SHOOT_SETTLE_MS);
            phase = Phase.SETTLE;
        }

        @Override public void loop() {
            long now = Clock.SYSTEM.nowNanos();
            switch (phase) {
                case SETTLE:
                    poseSettled = settle.settled(
                            settledAt(robot, TreeState.poseFor(shotSide), POSITION_TOLERANCE_IN), now);
                    if (!poseSettled && !settle.timedOut(now)) return;
                    if (!poseSettled) warn("Pose/heading/velocity settle timeout; planned batch will still be attempted");
                    // The wait window starts here, after the approach/turn,
                    // so the camera gets the bounded lineup wait from this side.
                    visionStart = now;
                    phase = Phase.VISION;
                    break;
                case VISION:
                    // Plain rule on the latest result: any alliance tag
                    // shoots from this side. The result may be a stale
                    // pre-arrival frame; that is the accepted risk of the
                    // plain rule (no freshness/boundary gate).
                    tagSeen = robot.vision.anyAllianceFiducialLatest();
                    VisionDecision decision = decideVisionAction(tagSeen, elapsedMs(now, visionStart),
                            poseSettled, switched,
                            robot.getPose().distance(TreeState.poseFor(shotSide)));
                    if (decision == VisionDecision.WAIT) return;
                    if (decision == VisionDecision.SWITCH) {
                        sideSwitch = switchSides(robot, shotSide);
                        phase = Phase.SWITCH;
                        return;
                    }
                    if (decision == VisionDecision.BLIND_FEED) {
                        warn(switched
                                ? "No alliance tag after side switch; attempting blind batch"
                                : "No alliance tag; unconfirmed pose prevents safe side switch; attempting planned batch");
                    }
                    beginFeed();
                    break;
                case SWITCH:
                    if (sideSwitch.execute() == Status.RUNNING) return;
                    if (!sideSwitch.reachedDestination()) {
                        state.abortControlPath(sideSwitch.failureReason());
                        warn("CONTROL PATH ABORTED: " + sideSwitch.failureReason());
                        phase = Phase.DONE;
                        return;
                    }
                    shotSide = TreeState.otherShootPosition(shotSide);
                    switched = true;
                    poseSettled = true;
                    // Recheck the latest result after the final measured turn,
                    // with a fresh bounded wait from the new side.
                    visionStart = now;
                    phase = Phase.VISION;
                    break;
                case FEED:
                    if (feed.execute() == Status.RUNNING) return;
                    if (feed.attempted()) {
                        boolean positionVerified = robot.getPose().distance(TreeState.poseFor(shotSide))
                                <= POSITION_TOLERANCE_IN;
                        state.recordAttempt(shotSide, feed.outcome(), poseSettled, tagSeen, positionVerified);
                        // Publish selection inputs together, so the reactive tree cannot
                        // preempt this leg between its batch and intake bookkeeping.
                        state.markIntakeDone(intakeArea);
                        robot.telemetry.addData("Tree batch result",
                                "%s; settled=%s tagSeen=%s; scores UNCONFIRMED",
                                feed.outcome(), poseSettled, tagSeen);
                    } else {
                        state.abortControlPath("Feed ended without a completed batch attempt: " + feed.outcome());
                    }
                    phase = Phase.DONE;
                    break;
                case DONE:
                    break;
            }
        }

        private void beginFeed() {
            poseSettled = poseSettled
                    && settledAt(robot, TreeState.poseFor(shotSide), POSITION_TOLERANCE_IN);
            if (!poseSettled) warn("Feeding a planned attempt without measured pose readiness");
            feed = ShooterCommands.shootAllPollen(robot);
            phase = Phase.FEED;
        }

        private void warn(String reason) {
            robot.telemetry.addData("Tree shooting WARNING", reason);
        }

        @Override public Status status() { return Status.of(phase != Phase.DONE); }

        @Override public void end(EndCondition condition) {
            if (condition == EndCondition.INTERRUPTED) {
                if (phase == Phase.SWITCH && sideSwitch != null) {
                    sideSwitch.invalidate();
                    state.abortControlPath("Side switch interrupted before verified destination");
                }
                if (feed != null) feed.invalidate();
                robot.follower.stop();
            }
            if (condition == EndCondition.INTERRUPTED || state.controlPathAborted()) {
                robot.follower.stop();
                try {
                    robot.transfer.stopImmediately();
                } finally {
                    robot.intake.stopImmediately();
                }
            }
        }
    }

    public enum SwitchOutcome { NOT_STARTED, RUNNING, REACHED, ABORTED, INTERRUPTED }

    /**
     * Owns follower control through turn-in, bounded corridor travel and turn-out.
     * No strict drivetrain command exceptions are caught or converted into success.
     */
    public static SideSwitchCommand switchSides(Robot robot, Position fromSide) {
        return new SideSwitchCommand(robot.follower, fromSide,
                reason -> robot.telemetry.addData("Tree side switch ABORTED", reason), Clock.SYSTEM);
    }

    public static final class SideSwitchCommand extends Command {
        private final Follower follower;
        private final Consumer<String> warning;
        private final Clock clock;
        private final Pose fromPose;
        private final Pose toPose;
        private final double corridorHeading;
        private final Path path;
        private SwitchOutcome outcome = SwitchOutcome.NOT_STARTED;
        private String failure = "";
        private int stage;
        private Pose turnPose;
        private SettleWindow settle;
        private long pathRunId;

        SideSwitchCommand(Follower follower, Position fromSide, Consumer<String> warning, Clock clock) {
            this.follower = follower;
            this.warning = warning;
            this.clock = clock;
            fromPose = pedroPose(TreeState.poseFor(fromSide));
            toPose = pedroPose(TreeState.poseFor(TreeState.otherShootPosition(fromSide)));
            Pose wpFrom = fromSide == Position.GARDEN_SHOOT ? PathPoses.underHiveSouth : PathPoses.underHiveNorth;
            Pose wpTo = fromSide == Position.GARDEN_SHOOT ? PathPoses.underHiveNorth : PathPoses.underHiveSouth;
            corridorHeading = Math.atan2(wpTo.y() - wpFrom.y(), wpTo.x() - wpFrom.x());
            path = Paths.through(fromPose, wpFrom, wpTo, toPose)
                    .heading(Interpolator.constant(corridorHeading));
            addRequirements(follower);
        }

        public SwitchOutcome outcome() { return outcome; }
        public boolean reachedDestination() { return outcome == SwitchOutcome.REACHED; }
        public String failureReason() { return failure; }

        @Override public void start() {
            outcome = SwitchOutcome.RUNNING;
            failure = "";
            stage = 0;
            if (!(follower.localizer.pose().distance(fromPose) <= SWITCH_POSITION_TOLERANCE_IN)) {
                abort("Side switch start pose not reached");
                return;
            }
            turnPose = fromPose.withHeading(corridorHeading);
            follower.hold(turnPose);
            settle = new SettleWindow(clock.nowNanos(), Timings.flowerTurnTimeoutMs);
        }

        @Override public void loop() {
            if (outcome != SwitchOutcome.RUNNING) return;
            long now = clock.nowNanos();
            if (stage == 1) {
                // No follower.idle() check here: the follower goes IDLE on
                // natural path completion (update() calls stop() when the
                // path tracker finishes with holdEnd=false), so an idle
                // check would abort the whole auto on the corridor leg's
                // normal arrival. A genuinely replaced control still aborts
                // via pathRunId/manual; a mid-corridor stop aborts via the
                // "parametric end without measured destination arrival"
                // check below.
                if (follower.pathRunId() != pathRunId || follower.manual()) {
                    abort("Side switch corridor control replaced");
                    return;
                }
                // Parametric end only changes control to endpoint hold; it never authorizes shooting.
                if (follower.hasPathProgress() && follower.atParametricEnd()) {
                    if (!(follower.localizer.pose().distance(toPose) <= POSITION_TOLERANCE_IN)) {
                        abort("Side switch parametric end without measured destination arrival");
                        return;
                    }
                    follower.hold(toPose.withHeading(corridorHeading));
                    stage = 2;
                    settle = new SettleWindow(now, SHOOT_SETTLE_MS);
                } else if (settle.timedOut(now)) {
                    abort("Side switch corridor traversal timed out");
                }
                return;
            }
            Pose target = stage == 0 ? turnPose
                    : stage == 2 ? toPose.withHeading(corridorHeading) : toPose;
            if (settle.settled(settledAt(follower, target, SWITCH_POSITION_TOLERANCE_IN), now)) {
                if (stage == 0) {
                    follower.follow(path);
                    pathRunId = follower.pathRunId();
                    stage = 1;
                    settle = new SettleWindow(now, SWITCH_TRAVERSE_TIMEOUT_MS);
                } else if (stage == 2) {
                    follower.hold(toPose);
                    stage = 3;
                    settle = new SettleWindow(now, Timings.flowerTurnTimeoutMs);
                } else {
                    outcome = SwitchOutcome.REACHED;
                }
            } else if (settle.timedOut(now)) {
                abort("Side switch did not physically settle at stage " + stage);
            }
        }

        private void abort(String reason) {
            failure = reason;
            outcome = SwitchOutcome.ABORTED;
            follower.stop();
            warning.accept(reason);
        }

        @Override public Status status() { return Status.of(outcome == SwitchOutcome.RUNNING); }

        @Override public void end(EndCondition condition) {
            if (condition == EndCondition.INTERRUPTED) {
                failure = "Side switch interrupted";
                outcome = SwitchOutcome.INTERRUPTED;
                follower.stop();
            }
        }
    }

    /** Reposition only commits the new tree position after measured arrival. */
    public static Command reposition(Robot robot, TreeState state) {
        return new Command() {
            private SideSwitchCommand inner;
            private boolean done;
            private Position target;

            { addRequirements(robot.follower); }

            @Override public void start() {
                done = state.controlPathAborted();
                inner = null;
                if (done) return;
                target = TreeState.otherShootPosition(state.position());
                inner = switchSides(robot, state.position());
            }

            @Override public void loop() {
                if (done || inner.execute() == Status.RUNNING) return;
                if (inner.reachedDestination()) state.setPosition(target);
                else state.abortControlPath(inner.failureReason());
                done = true;
            }

            @Override public Status status() { return Status.of(!done); }

            @Override public void end(EndCondition condition) {
                if (condition == EndCondition.INTERRUPTED && inner != null && !done) {
                    inner.invalidate();
                    state.abortControlPath("Reposition interrupted before verified destination");
                }
            }
        };
    }
}
