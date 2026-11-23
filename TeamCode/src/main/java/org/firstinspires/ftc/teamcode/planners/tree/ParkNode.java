package org.firstinspires.ftc.teamcode.planners.tree;

import com.rmsftc.bttree.BehaviorNode;
import com.rmsftc.bttree.NodeStatus;
import com.rmsftc.utils.blackboard.Blackboard;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The tree auto's park action: calls {@code robot.moveToParkZone()} with
 * the same 0.5 s retry semantics the plan-based autos use in onLoop (retry
 * while it returns false; once it returns true the park path is being
 * followed and it is not called again). RUNNING until
 * {@code robot.inParkZone()}, then SUCCESS. The follower still owns its
 * route and endpoint hold; zone overlap does not stop the autonomous.
 */
public class ParkNode extends BehaviorNode {
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    private final BooleanSupplier inParkZone;
    private final BooleanSupplier moveToParkZone;
    private final DoubleSupplier timeSeconds;
    private boolean handoffPending;
    private boolean parkStarted;
    private double lastAttemptTimeSec;

    public ParkNode(String name, Blackboard board, Robot robot) {
        this(name, board, robot::inParkZone, robot::moveToParkZone, robot::getOpModeTimeSeconds);
    }

    ParkNode(String name, Blackboard board, BooleanSupplier inParkZone,
             BooleanSupplier moveToParkZone, DoubleSupplier timeSeconds) {
        super(name, board);
        this.inParkZone = inParkZone;
        this.moveToParkZone = moveToParkZone;
        this.timeSeconds = timeSeconds;
    }

    @Override
    protected void onStart() {
        handoffPending = true;
        parkStarted = false;
        lastAttemptTimeSec = Double.NEGATIVE_INFINITY;
    }

    @Override
    protected NodeStatus onTick() {
        // ReactiveSelector invalidates the outgoing leg after ticking this branch.
        // Let that cleanup finish before granting parking follower ownership.
        if (handoffPending) {
            handoffPending = false;
            return NodeStatus.RUNNING;
        }
        if (inParkZone.getAsBoolean()) {
            return NodeStatus.SUCCESS;
        }
        if (!parkStarted) {
            double now = timeSeconds.getAsDouble();
            if (now - lastAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastAttemptTimeSec = now;
                parkStarted = moveToParkZone.getAsBoolean();
            }
        }
        return NodeStatus.RUNNING;
    }
}
