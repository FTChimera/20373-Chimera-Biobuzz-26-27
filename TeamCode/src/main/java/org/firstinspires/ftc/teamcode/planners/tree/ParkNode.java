package org.firstinspires.ftc.teamcode.planners.tree;

import com.rmsftc.bttree.BehaviorNode;
import com.rmsftc.bttree.NodeStatus;
import com.rmsftc.utils.blackboard.Blackboard;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;

/**
 * The tree auto's park action: calls {@code robot.moveToParkZone()} with
 * the same 0.5 s retry semantics the plan-based autos use in onLoop (retry
 * while it returns false; once it returns true the park path is being
 * followed and it is not called again). RUNNING until
 * {@code robot.inParkZone()}, then SUCCESS — at which point the OpMode's
 * onLoop stop check ends the auto.
 */
public class ParkNode extends BehaviorNode {
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    private final Robot robot;
    private boolean parkStarted;
    private double lastAttemptTimeSec;

    public ParkNode(String name, Blackboard board, Robot robot) {
        super(name, board);
        this.robot = robot;
    }

    @Override
    protected void onStart() {
        parkStarted = false;
        lastAttemptTimeSec = Double.NEGATIVE_INFINITY;
    }

    @Override
    protected NodeStatus onTick() {
        if (robot.inParkZone()) {
            return NodeStatus.SUCCESS;
        }
        if (!parkStarted) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastAttemptTimeSec = now;
                parkStarted = robot.moveToParkZone();
            }
        }
        return NodeStatus.RUNNING;
    }
}
