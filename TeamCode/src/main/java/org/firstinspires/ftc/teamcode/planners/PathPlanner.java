package org.firstinspires.ftc.teamcode.planners;

import androidx.annotation.NonNull;

import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.Objects;

/**
 * One leg of an autonomous plan, as a BehaviorArchitectures
 * {@link Command}. A planner knows where it starts ({@link #from()}),
 * where it ends ({@link #to()} and {@link #getEndPose()}), how to build
 * its paths from the start pose ({@link #buildPaths()}), and the command
 * sequence that drives the leg ({@link #buildCommand()}).
 *
 * <p>Lifecycle: {@link Plan} sets {@link #startPose} (the previous leg's
 * end pose) before scheduling. On the first tick, {@link #start()} calls
 * {@code buildPaths()} then {@code buildCommand()}; {@code loop()},
 * {@code status()} and {@code end()} forward to that inner command.
 * The inner command is executed inline (never scheduled), so this
 * planner declares the union of the inner command's requirement tokens
 * in its constructor — the scheduler enforces conflicts at the planner
 * level.
 */
public abstract class PathPlanner extends Command {
    protected final Robot robot;
    protected Pose2d startPose;

    private Command inner;
    private Status innerStatus = Status.RUNNING;

    protected PathPlanner(Robot robot, Object... tokens) {
        this.robot = robot;
        addRequirements(tokens);
    }

    /** The pose this leg ends at (the next leg's start pose). */
    public abstract Pose2d getEndPose();

    /** The position this leg starts from. */
    public abstract Position from();

    /** The position this leg ends at. */
    public abstract Position to();

    /** Builds this leg's paths from {@link #startPose}. Called in {@link #start()}. */
    protected abstract void buildPaths();

    /** The leg's command sequence. Called in {@link #start()}, after {@link #buildPaths()}. */
    protected abstract Command buildCommand();

    /** Sets the pose this leg starts from. Called by {@link Plan} before scheduling. */
    public final void setStartPose(Pose2d newPose) {
        this.startPose = Objects.requireNonNull(newPose, "Start pose cannot be null");
    }

    @Override
    public final void start() {
        buildPaths();
        inner = buildCommand();
        innerStatus = Status.RUNNING;
    }

    @Override
    public final void loop() {
        innerStatus = inner.execute();
    }

    @Override
    public final Status status() {
        return innerStatus;
    }

    @Override
    public final void end(EndCondition endCondition) {
        // On a natural finish the inner command already ended itself in
        // its last execute(); only an interrupt needs forwarding.
        if (endCondition == EndCondition.INTERRUPTED && inner != null) {
            inner.invalidate();
        }
    }

    @NonNull
    @Override
    public abstract String toString();
}
