package org.firstinspires.ftc.teamcode.planners.commands;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.FunctionalCommand;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Drivetrain commands on BehaviorArchitectures. The follower is the
 * requirement token for drive commands (as in the Ivy versions).
 * Follow/turn commands are fire-and-forget into the follower: their
 * {@code end()} deliberately does NOT stop the follower, so a timeout
 * or interrupt leaves the follower driving its last path until the
 * next follow/hold/stop replaces it — exactly what the Ivy versions
 * did (they had no end action).
 */
public class DrivetrainCommands {

    /** Follows a path until the follower reaches its parametric end. */
    private static class FollowCommand extends Command {
        private final Follower follower;
        private final Supplier<Path> pathSupplier;

        FollowCommand(Follower follower, Supplier<Path> pathSupplier) {
            this.follower = follower;
            this.pathSupplier = pathSupplier;
            addRequirements(follower);
        }

        @Override
        public void start() {
            follower.follow(Objects.requireNonNull(pathSupplier.get(), "Path supplier returned null"));
        }

        @Override
        public void loop() {
        }

        @Override
        public Status status() {
            return Status.of(!follower.atParametricEnd());
        }

        @Override
        public void end(EndCondition endCondition) {
            // No-op: the follower keeps its last follow() call.
        }
    }

    /** Holds the current position while turning to a heading. */
    private static class TurnCommand extends Command {
        private final Follower follower;
        private final double targetHeadingRadians;

        TurnCommand(Follower follower, double targetHeadingRadians) {
            this.follower = follower;
            this.targetHeadingRadians = targetHeadingRadians;
            addRequirements(follower);
        }

        @Override
        public void start() {
            Pose current = follower.localizer.pose();
            follower.hold(new Pose(current.x(), current.y(), targetHeadingRadians));
        }

        @Override
        public void loop() {
        }

        @Override
        public Status status() {
            return Status.of(follower.isBusy());
        }

        @Override
        public void end(EndCondition endCondition) {
            // No-op: the follower keeps its last hold() call.
        }
    }

    public static Command follow(Follower follower, Path path) {
        return follow(follower, () -> path);
    }

    public static Command follow(Follower follower, Supplier<Path> pathSupplier) {
        return new FollowCommand(follower, pathSupplier);
    }

    /**
     * Like {@link #follow}, but also finishes after {@code timeoutMs} even if
     * the path end was never reached. Use for legs where the robot may be
     * physically blocked from the path end (e.g. pushing into a flower with
     * the deployer disabled): without this, one blocked leg hangs the whole
     * auto. The follower keeps driving during whatever step comes next
     * until that step replaces the follow, which is fine for an intake dwell.
     */
    public static Command followWithTimeout(Follower follower, Supplier<Path> pathSupplier, double timeoutMs) {
        return follow(follower, pathSupplier).withTimeout((long) timeoutMs);
    }

    /**
     * Turns the robot in place to face {@code targetHeadingRadians},
     * holding its current position. Done when the follower's hold
     * completes (heading within tolerance and stopped). Put this before
     * a constant-heading follow so the robot is already facing the drive
     * direction instead of turning while it drives, and after one to
     * face the next action (e.g. the shoot pose) before it starts.
     */
    public static Command turnToHeading(Follower follower, double targetHeadingRadians) {
        return new TurnCommand(follower, targetHeadingRadians);
    }

    /**
     * Like {@link #turnToHeading}, but also finishes after {@code timeoutMs}
     * even if the hold never settles (e.g. the robot is wedged against a
     * flower and cannot reach the heading tolerance). The next follow's
     * heading interpolator corrects any leftover heading error.
     */
    public static Command turnToHeadingWithTimeout(Follower follower, double targetHeadingRadians, double timeoutMs) {
        return turnToHeading(follower, targetHeadingRadians).withTimeout((long) timeoutMs);
    }

    /**
     * Runs {@code action} once when the follower's path completion reaches
     * {@code tValue} (0-1 along the current path), while a {@link #follow}
     * command drives the path in parallel. Ends when the path ends. Like
     * {@link #setMaxPowerOnFly}, this does not require the follower, so it
     * can run in parallel with a follow command. List the follow command
     * first in the parallel group so the path is started before this first
     * executes.
     */
    public static Command runAtCompletion(Robot robot, double tValue, Runnable action) {
        AtomicBoolean done = new AtomicBoolean(false);
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    if (!done.get() && robot.follower.completion() >= tValue) {
                        done.set(true);
                        action.run();
                    }
                },
                () -> Status.of(!done.get()),
                endCondition -> {
                });
    }

    public static Command setMaxPowerOnFly(Robot robot, double tValue, double newPower) {
        AtomicReference<Double> previousPower = new AtomicReference<>();
        return new FunctionalCommand(
                () -> previousPower.set(Constants.FOLLOWER_MAX_PATH_SPEED.get()),
                () -> {
                    if (robot.follower.completion() >= tValue) Constants.FOLLOWER_MAX_PATH_SPEED.set(newPower);
                },
                () -> Status.of(!robot.follower.atParametricEnd()),
                endCondition -> {
                    Double priorSpeed = previousPower.get();
                    if (priorSpeed != null) {
                        Constants.FOLLOWER_MAX_PATH_SPEED.set(priorSpeed);
                    }
                });
    }
}
