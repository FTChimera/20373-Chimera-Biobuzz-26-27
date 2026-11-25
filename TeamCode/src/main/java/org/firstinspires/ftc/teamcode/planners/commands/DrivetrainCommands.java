package org.firstinspires.ftc.teamcode.planners.commands;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;

import java.util.Objects;
import com.rmsftc.utils.Clock;
import java.util.function.Supplier;

/** Drive commands own the follower until completion; cancellation stops it immediately. */
public class DrivetrainCommands {

    static class FollowCommand extends Command {
        private final Follower follower;
        private final Supplier<Path> pathSupplier;
        private final double timeoutMs;
        private final double dwellMs;
        private final boolean stopOnCompletion;
        private final boolean strict;
        private final double[] tolerances;
        private final Clock clock;
        private Pose target;
        private long runId;
        private long started;
        private long dwellStarted;
        private boolean dwelling;
        private boolean done;
        private boolean degraded; // timed out or control replaced: end() must stop, not hold the target

        FollowCommand(Follower follower, Supplier<Path> pathSupplier, double timeoutMs,
                      double dwellMs, boolean stopOnCompletion, boolean strict,
                      double[] tolerances, Clock clock) {
            this.follower = Objects.requireNonNull(follower);
            this.pathSupplier = Objects.requireNonNull(pathSupplier);
            this.timeoutMs = timeoutMs;
            this.dwellMs = dwellMs;
            this.stopOnCompletion = stopOnCompletion;
            this.strict = strict;
            this.tolerances = tolerances;
            this.clock = Objects.requireNonNull(clock);
            addRequirements(follower);
        }

        @Override public void start() {
            done = false;
            dwelling = false;
            degraded = false;
            started = clock.nowNanos();
            boolean initialized = false;
            try {
                Path path = Objects.requireNonNull(pathSupplier.get(), "Path supplier returned null");
                target = path.endPose();
                follower.follow(path);
                runId = follower.pathRunId();
                initialized = true;
            } finally {
                if (!initialized) follower.stop();
            }
        }

        @Override public void loop() {
            if (done) return;
            boolean pathCompleted = follower.hasPathProgress() && follower.completion() >= 1;
            if (follower.pathRunId() != runId || follower.manual() || (follower.idle() && !pathCompleted)) {
                // Control was taken over (or the path ended early): stop and
                // degrade — complete the command so the sequence continues
                // from wherever the robot actually is, instead of killing
                // the whole OpMode with an exception.
                follower.stop();
                degraded = true;
                done = true;
                return;
            }
            if (strict && follower.idle() && pathCompleted) follower.hold(target);
            long now = clock.nowNanos();
            if (!dwelling) {
                boolean ready = follower.hasPathProgress() && follower.atParametricEnd()
                        && (!strict || follower.isPoseSettled(target,
                        tolerances[0], tolerances[1], tolerances[2], tolerances[3]));
                if (!ready && elapsedMs(now, started) < timeoutMs) return;
                if (!ready && strict) {
                    // Timeout: stop and degrade — complete the command so the
                    // sequence continues from the actual pose, instead of
                    // killing the whole OpMode with an exception.
                    follower.stop();
                    degraded = true;
                    done = true;
                    return;
                }
                dwelling = true;
                dwellStarted = now;
            }
            done = elapsedMs(now, dwellStarted) >= dwellMs;
        }

        @Override public Status status() { return Status.of(!done); }

        @Override public void end(EndCondition condition) {
            follower.finishPathCommand(runId);
            if (degraded || condition == EndCondition.INTERRUPTED || stopOnCompletion
                    || !follower.atParametricEnd()) {
                follower.stop();
            } else if (strict) {
                follower.hold(target);
            }
        }
    }

    static class TurnCommand extends Command {
        private final Follower follower;
        private final double heading;
        private final double timeoutMs;
        private final double[] tolerances;
        private final Clock clock;
        private long started;
        private boolean ready;

        TurnCommand(Follower follower, double heading, double timeoutMs,
                    double[] tolerances, Clock clock) {
            this.follower = Objects.requireNonNull(follower);
            if (!Double.isFinite(heading)) throw new IllegalArgumentException("Heading must be finite");
            this.heading = heading;
            this.timeoutMs = timeoutMs;
            this.tolerances = tolerances;
            this.clock = Objects.requireNonNull(clock);
            addRequirements(follower);
        }

        @Override public void start() {
            ready = false;
            started = clock.nowNanos();
            Pose current = follower.localizer.pose();
            follower.hold(new Pose(current.x(), current.y(), heading));
        }

        @Override public void loop() {
            ready = follower.holding() && follower.hasControlUpdate()
                    && follower.isHeadingSettled(heading, tolerances[0], tolerances[1], tolerances[2]);
            if (!ready && elapsedMs(clock.nowNanos(), started) >= timeoutMs) {
                // Timeout: stop and degrade — complete the turn so the
                // sequence continues from the actual heading, instead of
                // killing the whole OpMode with an exception.
                follower.stop();
                ready = true;
            }
        }

        @Override public Status status() { return Status.of(!ready); }

        @Override public void end(EndCondition condition) {
            if (condition == EndCondition.INTERRUPTED) follower.stop();
        }
    }

    public static Command follow(Follower follower, Path path) {
        return follow(follower, () -> path);
    }

    /** Parametric completion only, not a safety-critical settled-pose gate. */
    public static Command follow(Follower follower, Supplier<Path> pathSupplier) {
        return new FollowCommand(follower, pathSupplier, Double.POSITIVE_INFINITY,
                0, false, false, null, Clock.SYSTEM);
    }

    /** Best-effort path: stops on timeout. For continued pushing use {@link #followForIntake}. */
    public static Command followWithTimeout(Follower follower, Supplier<Path> pathSupplier, double timeoutMs) {
        requireDuration(timeoutMs);
        return new FollowCommand(follower, pathSupplier, timeoutMs, 0, false, false, null, Clock.SYSTEM);
    }

    /**
     * Keeps path control during a bounded approach and subsequent intake dwell.
     * The dwell begins on parametric completion or approach timeout. Owns the
     * follower throughout; both cancellation and dwell completion stop motion.
     */
    public static Command followForIntake(Follower follower, Supplier<Path> pathSupplier,
                                         double timeoutMs, double dwellMs) {
        return followForIntake(follower, pathSupplier, timeoutMs, dwellMs, Clock.SYSTEM);
    }

    static Command followForIntake(Follower follower, Supplier<Path> pathSupplier,
                                  double timeoutMs, double dwellMs, Clock clock) {
        requireDuration(timeoutMs);
        requireDuration(dwellMs);
        return new FollowCommand(follower, pathSupplier, timeoutMs, dwellMs, true, false, null, clock);
    }

    /** Fails explicitly and stops on timeout; never authorizes a side switch on timeout alone. */
    public static Command followSettledWithTimeout(Follower follower, Supplier<Path> pathSupplier,
                                                   double timeoutMs) {
        return followSettledWithTimeout(follower, pathSupplier, timeoutMs, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.POSITION_TOLERANCE_IN,
                org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.HEADING_TOLERANCE_RAD, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.LINEAR_VELOCITY_TOLERANCE_IN_PER_SEC, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.ANGULAR_VELOCITY_TOLERANCE_RAD_PER_SEC);
    }

    public static Command followSettledWithTimeout(Follower follower, Supplier<Path> pathSupplier,
                                                   double timeoutMs, double positionTolerance,
                                                   double headingTolerance, double linearVelocityTolerance,
                                                   double angularVelocityTolerance) {
        requireDuration(timeoutMs);
        double[] tolerances = tolerances(positionTolerance, headingTolerance,
                linearVelocityTolerance, angularVelocityTolerance);
        return new FollowCommand(follower, pathSupplier, timeoutMs, 0, false, true, tolerances, Clock.SYSTEM);
    }

    public static Command turnToHeading(Follower follower, double targetHeadingRadians) {
        return new TurnCommand(follower, targetHeadingRadians, Double.POSITIVE_INFINITY,
                tolerances(org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.HEADING_TOLERANCE_RAD, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.LINEAR_VELOCITY_TOLERANCE_IN_PER_SEC, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.ANGULAR_VELOCITY_TOLERANCE_RAD_PER_SEC),
                Clock.SYSTEM);
    }

    /** Measured heading and both velocities must settle; timeout stops and throws. */
    public static Command turnToHeadingWithTimeout(Follower follower, double targetHeadingRadians,
                                                   double timeoutMs) {
        return turnToHeadingWithTimeout(follower, targetHeadingRadians, timeoutMs,
                org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.HEADING_TOLERANCE_RAD, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.LINEAR_VELOCITY_TOLERANCE_IN_PER_SEC, org.firstinspires.ftc.teamcode.planners.Constants.Drivetrain.ANGULAR_VELOCITY_TOLERANCE_RAD_PER_SEC);
    }

    public static Command turnToHeadingWithTimeout(Follower follower, double targetHeadingRadians,
                                                   double timeoutMs, double headingTolerance,
                                                   double linearVelocityTolerance, double angularVelocityTolerance) {
        requireDuration(timeoutMs);
        return new TurnCommand(follower, targetHeadingRadians, timeoutMs,
                tolerances(headingTolerance, linearVelocityTolerance, angularVelocityTolerance), Clock.SYSTEM);
    }

    /**
     * List follow first in a parallel group. A missed threshold is skipped when
     * that path command ends (including timeout), rather than blocking the group.
     */
    public static Command runAtCompletion(Robot robot, double tValue, Runnable action) {
        return runAtCompletion(robot.follower, tValue, action);
    }

    public static Command runAtCompletion(Follower follower, double tValue, Runnable action) {
        Objects.requireNonNull(follower);
        Objects.requireNonNull(action);
        if (!Double.isFinite(tValue) || tValue < 0 || tValue > 1) {
            throw new IllegalArgumentException("Path completion threshold must be in [0, 1]");
        }
        return new Command() {
            private boolean done;
            private long runId;

            @Override public void start() {
                done = false;
                runId = follower.pathRunId();
            }

            @Override public void loop() {
                if (done) return;
                if (runId != follower.pathRunId()) {
                    done = true;
                    return;
                }
                if (follower.hasPathProgress() && follower.completion() >= tValue) {
                    done = true;
                    boolean completed = false;
                    try {
                        action.run();
                        completed = true;
                    } finally {
                        if (!completed) follower.stop();
                    }
                } else if (follower.pathRunFinished()) {
                    done = true;
                }
            }

            @Override public Status status() { return Status.of(!done); }
            @Override public void end(EndCondition condition) { }
        };
    }

    public static Command setMaxPowerOnFly(Robot robot, double tValue, double newPower) {
        if (!Double.isFinite(newPower) || newPower < 0 || newPower > 1) {
            throw new IllegalArgumentException("Path power must be in [0, 1]");
        }
        return new Command() {
            private Double previousPower;
            private Command trigger;

            @Override public void start() {
                previousPower = Constants.FOLLOWER_MAX_PATH_SPEED.get();
                trigger = runAtCompletion(robot.follower, tValue,
                        () -> Constants.FOLLOWER_MAX_PATH_SPEED.set(newPower));
            }

            @Override public void loop() {
                if (trigger != null && !trigger.execute().isRunning()) trigger = null;
            }

            @Override public Status status() {
                return Status.of(trigger != null || !robot.follower.pathRunFinished());
            }

            @Override public void end(EndCondition condition) {
                if (trigger != null) trigger.invalidate();
                if (previousPower != null) Constants.FOLLOWER_MAX_PATH_SPEED.set(previousPower);
            }
        };
    }

    private static double elapsedMs(long now, long then) { return (now - then) / 1_000_000.0; }

    private static void requireDuration(double duration) {
        if (!Double.isFinite(duration) || duration < 0) {
            throw new IllegalArgumentException("Duration must be finite and nonnegative");
        }
    }

    private static double[] tolerances(double... values) {
        for (double value : values) requireDuration(value);
        return values;
    }
}
