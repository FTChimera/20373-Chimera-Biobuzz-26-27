package org.firstinspires.ftc.teamcode.planners.tree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.rmsftc.bttree.BehaviorNode;
import com.rmsftc.bttree.NodeStatus;
import com.rmsftc.bttree.nodes.ConditionNode;
import com.rmsftc.bttree.nodes.ReactiveSelectorNode;
import com.rmsftc.bttree.nodes.SequenceNode;
import com.rmsftc.utils.blackboard.Blackboard;

import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class ParkNodeTest {
    @Test
    public void parkingStartsOnlyAfterOutgoingLegCleanup() {
        Blackboard board = new Blackboard();
        AtomicBoolean shouldPark = new AtomicBoolean();
        AtomicBoolean driving = new AtomicBoolean();
        AtomicBoolean interrupted = new AtomicBoolean();
        AtomicInteger parkAttempts = new AtomicInteger();
        ParkNode park = new ParkNode("Park", board, () -> false, () -> {
            parkAttempts.incrementAndGet();
            driving.set(true);
            return true;
        }, () -> 0);
        BehaviorNode outgoingLeg = new BehaviorNode("Leg", board) {
            @Override protected NodeStatus onTick() {
                driving.set(true);
                return NodeStatus.RUNNING;
            }

            @Override protected void onEnd() {
                interrupted.set(true);
                driving.set(false);
            }
        };
        BehaviorNode root = new ReactiveSelectorNode("Root", board, Arrays.asList(
                new SequenceNode("Park branch", board, Arrays.asList(
                        new ConditionNode("Should park", board, shouldPark::get), park)),
                outgoingLeg));

        assertEquals(NodeStatus.RUNNING, root.tick());
        assertTrue(driving.get());
        shouldPark.set(true);
        assertEquals(NodeStatus.RUNNING, root.tick());
        assertTrue(interrupted.get());
        assertFalse(driving.get());
        assertEquals(0, parkAttempts.get());

        assertEquals(NodeStatus.RUNNING, root.tick());
        assertTrue(driving.get());
        assertEquals(1, parkAttempts.get());
        root.tick();
        assertTrue(driving.get());
        assertEquals(1, parkAttempts.get());
    }

    @Test
    public void unsuccessfulParkingRetriesAtHalfSecondIntervals() {
        double[] now = {0};
        AtomicInteger attempts = new AtomicInteger();
        ParkNode park = new ParkNode("Park", new Blackboard(), () -> false,
                () -> attempts.incrementAndGet() >= 2, () -> now[0]);
        park.tick();
        assertEquals(0, attempts.get());
        park.tick();
        assertEquals(1, attempts.get());
        now[0] = 0.49;
        park.tick();
        assertEquals(1, attempts.get());
        now[0] = 0.5;
        park.tick();
        assertEquals(2, attempts.get());
        now[0] = 2;
        park.tick();
        assertEquals(2, attempts.get());
    }

    @Test
    public void zoneOverlapDoesNotReplaceAnExistingParkRoute() {
        AtomicInteger attempts = new AtomicInteger();
        ParkNode park = new ParkNode("Park", new Blackboard(), () -> true, () -> {
            attempts.incrementAndGet();
            return true;
        }, () -> 0);
        assertEquals(NodeStatus.RUNNING, park.tick());
        assertEquals(NodeStatus.SUCCESS, park.tick());
        assertEquals(0, attempts.get());
    }
}
