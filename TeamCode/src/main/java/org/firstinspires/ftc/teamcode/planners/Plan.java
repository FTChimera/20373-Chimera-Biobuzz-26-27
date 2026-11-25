package org.firstinspires.ftc.teamcode.planners;

import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import com.rmsftc.utils.math.Pose;

import java.util.Objects;

/**
 * Builds an autonomous plan from planner legs: validates the
 * {@link Position} chain (each leg's {@code from()} must match the
 * previous leg's {@code to()}), threads each leg's start pose from the
 * previous leg's end pose, and returns the legs as one sequential
 * command. Replaces the old PlanBuilder fluent API and PathPlannerList:
 * legs are passed explicitly, in order, as plain constructor calls.
 *
 * <pre>
 * Command plan = Plan.sequence(Position.GARDEN_START, Constants.StartPoses.gardenArea,
 *         new FromStartGardenPose.ShootPreloads(robot),
 *         new FromShootGardenPose.ToIntakeGarden(robot));
 * manager.schedule(plan);
 * </pre>
 *
 * Position-neutral legs ({@link Position#ANY}: waits, custom paths) are
 * skipped by validation and pass the current position through.
 */
public final class Plan {

    private Plan() {
    }

    /**
     * Sequences {@code legs} starting at {@code startPose}, validating
     * that consecutive legs chain by {@link Position}. The first
     * non-neutral leg's {@code from()} is not checked (no start position
     * given); use the overload below to check it too.
     *
     * @throws IllegalStateException if two consecutive legs do not chain
     */
    public static Command sequence(Pose startPose, PathPlanner... legs) {
        return sequence(null, startPose, legs);
    }

    /**
     * Sequences {@code legs} starting at {@code startPosition} /
     * {@code startPose}, validating the whole {@link Position} chain
     * including the first leg.
     *
     * @throws IllegalStateException if the legs do not chain from
     *                               {@code startPosition}
     */
    public static Command sequence(Position startPosition, Pose startPose, PathPlanner... legs) {
        Objects.requireNonNull(startPose, "Start pose cannot be null");
        if (legs.length == 0) {
            throw new IllegalArgumentException("A plan needs at least one leg");
        }
        Position current = startPosition;
        Pose prevEndPose = startPose;
        for (PathPlanner leg : legs) {
            if (leg.from() != Position.ANY) {
                if (current != null && leg.from() != current) {
                    throw new IllegalStateException(
                            "Plan leg " + leg + " starts from " + leg.from()
                                    + " but the previous leg ended at " + current);
                }
                current = leg.from();
            }
            if (leg.to() != Position.ANY) {
                current = leg.to();
            }
            leg.setStartPose(prevEndPose);
            prevEndPose = leg.getEndPose();
        }
        return Commands.sequence(legs);
    }
}
