package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.junit.Test;

public class FollowerReadinessTest {
    @Test public void holdResetsActualForesightBusyState() {
        FollowerTestRig rig = new FollowerTestRig();
        Foresight foresight = new Foresight(new ForesightConfig(c -> {
            c.naturalForwardDeceleration.set(24.0);
            c.naturalStrafeDeceleration.set(48.0);
            c.forwardTranslational.set(Controller.proportional(1));
            c.strafeTranslational.set(Controller.proportional(1));
            c.headingFeedback.set(Controller.proportional(1));
            c.brake.set(Controller.proportional(1));
            c.coast.set(Controller.proportional(1));
        }));
        Follower follower = new Follower(rig.localizer, rig.drivetrain, foresight);
        assertFalse(foresight.isBusy());
        follower.hold(new Pose(0, 0, 1));
        assertTrue(foresight.isBusy());
        assertFalse(follower.hasControlUpdate());
    }

    @Test public void progressIsZeroUntilFirstUpdateAndResetsForEveryPath() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.follow(FollowerTestRig.path());
        assertEquals(1, rig.algorithm.completion(), 0);
        assertEquals(0, rig.follower.completion(), 0);
        assertEquals(0, rig.follower.parametricCompletion(), 0);
        assertFalse(rig.follower.hasPathProgress());
        rig.progress = 0.6;
        rig.follower.update(0.02);
        assertEquals(0.6, rig.follower.completion(), 0);
        long first = rig.follower.pathRunId();
        rig.follower.follow(FollowerTestRig.path());
        assertTrue(rig.follower.pathRunId() > first);
        assertEquals(0, rig.follower.completion(), 0);
        assertFalse(rig.follower.hasPathProgress());
    }

    @Test public void headingOverrideCannotTurnPathProgressIntoHoldCompletion() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.follow(FollowerTestRig.path());
        rig.follower.headingOverride = 1.0;
        rig.holdChangesCompletion = true;
        rig.progress = 0.3;
        rig.follower.update(0.02);
        assertEquals(0.3, rig.follower.completion(), 0);
        assertFalse(rig.follower.atParametricEnd());
    }

    @Test public void finalSegmentResetStillReportsCompletion() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.follow(FollowerTestRig.path());
        rig.finishSegment = true;
        rig.follower.update(0.02);
        assertTrue(rig.follower.atParametricEnd());
        assertEquals(1, rig.follower.completion(), 0);
        rig.follower.update(0.02);
        assertTrue(rig.follower.pathRunFinished());
        assertEquals(1, rig.follower.completion(), 0);
    }

    @Test public void settledReadinessUsesWrappedHeadingAndBothVelocities() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.pose = new Pose(0, 0, Math.PI - 0.001);
        assertTrue(rig.follower.isPoseSettled(new Pose(0, 0, -Math.PI), .1, .01, .1, .1));
        rig.velocity = new Velocity(0, 0, 1);
        assertFalse(rig.follower.isHeadingSettled(-Math.PI, .01, .1, .1));
        rig.velocity = new Velocity(0, 1, 0);
        assertFalse(rig.follower.isHeadingSettled(-Math.PI, .01, .1, .1));
        rig.velocity = Velocity.zero();
        assertFalse(rig.follower.isPoseSettled(new Pose(1, 0, -Math.PI), .1, .01, .1, .1));
    }

    @Test public void nonfiniteMeasurementsAreNotReady() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.velocity = new Velocity(0, 0, Double.NaN);
        assertFalse(rig.follower.isHeadingSettled(0, .01, .1, .1));
    }

    @Test public void stopImmediatelyStopsHardwareAndEndsProgressRun() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.follow(FollowerTestRig.path());
        rig.follower.stop();
        assertEquals(1, rig.stops);
        assertTrue(rig.follower.idle());
        assertTrue(rig.follower.pathRunFinished());
    }
}
