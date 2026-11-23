package org.firstinspires.ftc.teamcode.planners.commands;

import static org.junit.Assert.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import org.firstinspires.ftc.teamcode.pedro.FollowerTestRig;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class DrivetrainCommandsTest {
    @Test public void turnDoesNotTrustStaleBusyOrCompleteBeforeMeasurementUpdate() {
        FollowerTestRig rig = new FollowerTestRig();
        Command turn = DrivetrainCommands.turnToHeading(rig.follower, 1);
        assertTrue(turn.execute().isRunning());
        assertEquals(1, rig.resets);
        rig.busy = false; // Includes Foresight's internal 100 ms timeout.
        rig.follower.update(.2);
        assertTrue(turn.execute().isRunning());
        rig.pose = new Pose(0, 0, 1);
        rig.velocity = new Velocity(0, 0, 1);
        assertTrue(turn.execute().isRunning());
        rig.velocity = new Velocity(1, 0, 0);
        assertTrue(turn.execute().isRunning());
        rig.velocity = Velocity.zero();
        assertFalse(turn.execute().isRunning());
        assertTrue(rig.follower.holding());
    }

    @Test public void boundedTurnThrowsAndStopsRatherThanContinuingSequence() {
        FollowerTestRig rig = new FollowerTestRig();
        AtomicLong clock = new AtomicLong();
        AtomicInteger next = new AtomicInteger();
        Command turn = new DrivetrainCommands.TurnCommand(rig.follower, 1, 1000,
                new double[]{.01, .1, .1}, clock::get);
        Command sequence = Commands.sequence(turn, Commands.runOnce(next::incrementAndGet));
        assertTrue(sequence.execute().isRunning());
        clock.set(1_001_000_000);
        assertThrows(IllegalStateException.class, sequence::execute);
        assertTrue(rig.follower.idle());
        assertEquals(1, rig.stops);
        assertEquals(0, next.get());
    }

    @Test public void progressWaitsForUpdateAndCallbackCanBeReused() {
        FollowerTestRig rig = new FollowerTestRig();
        AtomicInteger calls = new AtomicInteger();
        Command parallel = Commands.parallel(
                DrivetrainCommands.follow(rig.follower, FollowerTestRig::path),
                DrivetrainCommands.runAtCompletion(rig.follower, .5, calls::incrementAndGet));
        for (int run = 1; run <= 2; run++) {
            assertTrue(parallel.execute().isRunning());
            assertEquals(run - 1, calls.get());
            rig.progress = .7;
            rig.follower.update(.02);
            assertTrue(parallel.execute().isRunning());
            assertEquals(run, calls.get());
            rig.atEnd = true;
            rig.follower.update(.02);
            assertFalse(parallel.execute().isRunning());
        }
    }

    @Test public void missedCallbackDoesNotHangParallelAfterApproachTimeout() {
        FollowerTestRig rig = new FollowerTestRig();
        AtomicInteger calls = new AtomicInteger();
        Command parallel = Commands.parallel(
                DrivetrainCommands.followWithTimeout(rig.follower, FollowerTestRig::path, 0),
                DrivetrainCommands.runAtCompletion(rig.follower, .9, calls::incrementAndGet));
        assertFalse(parallel.execute().isRunning());
        assertEquals(0, calls.get());
        assertTrue(rig.follower.idle());
    }

    @Test public void externalTimeoutDecoratorAlsoStopsAndReleasesMissedCallback() {
        FollowerTestRig rig = new FollowerTestRig();
        Command parallel = Commands.parallel(
                DrivetrainCommands.follow(rig.follower, FollowerTestRig::path).withTimeout(0),
                DrivetrainCommands.runAtCompletion(rig.follower, .9, () -> fail("Missed threshold")));
        assertFalse(parallel.execute().isRunning());
        assertTrue(rig.follower.idle());
        assertEquals(1, rig.stops);
    }

    @Test public void intakeKeepsPushingThroughTimeoutAndDwellThenStops() {
        FollowerTestRig rig = new FollowerTestRig();
        AtomicLong clock = new AtomicLong();
        Command intake = DrivetrainCommands.followForIntake(
                rig.follower, FollowerTestRig::path, 100, 200, clock::get);
        assertTrue(intake.execute().isRunning());
        clock.set(100_000_000);
        assertTrue(intake.execute().isRunning());
        assertTrue(rig.follower.following());
        clock.set(299_000_000);
        assertTrue(intake.execute().isRunning());
        clock.set(300_000_000);
        assertFalse(intake.execute().isRunning());
        assertTrue(rig.follower.idle());
        assertEquals(1, rig.stops);
    }

    @Test public void intakeCancellationDuringDwellStopsImmediately() {
        FollowerTestRig rig = new FollowerTestRig();
        Command intake = DrivetrainCommands.followForIntake(
                rig.follower, FollowerTestRig::path, 0, 200);
        assertTrue(intake.execute().isRunning());
        intake.invalidate();
        assertEquals(1, rig.stops);
        assertTrue(rig.follower.idle());
    }

    @Test public void intakeWithZeroDwellStillStopsAtNaturalCompletion() {
        FollowerTestRig rig = new FollowerTestRig();
        Command intake = DrivetrainCommands.followForIntake(
                rig.follower, FollowerTestRig::path, 1000, 0);
        assertTrue(intake.execute().isRunning());
        rig.progress = 1;
        rig.atEnd = true;
        rig.follower.update(.02);
        assertFalse(intake.execute().isRunning());
        assertTrue(rig.follower.idle());
    }

    @Test public void strictPathRejectsParametricEndUntilPoseAndBothVelocitiesSettle() {
        FollowerTestRig rig = new FollowerTestRig();
        Command path = DrivetrainCommands.followSettledWithTimeout(
                rig.follower, FollowerTestRig::path, 1000);
        assertTrue(path.execute().isRunning());
        rig.finishSegment = true;
        rig.follower.update(.02);
        assertTrue(path.execute().isRunning());
        rig.pose = new Pose(10, 0, .5);
        rig.follower.update(.02);
        assertTrue(path.execute().isRunning());
        rig.pose = new Pose(10, 0, 0);
        rig.velocity = new Velocity(0, 0, 1);
        assertTrue(path.execute().isRunning());
        rig.velocity = Velocity.zero();
        assertFalse(path.execute().isRunning());
    }

    @Test public void strictPathTimeoutCannotAuthorizeNextCommand() {
        FollowerTestRig rig = new FollowerTestRig();
        AtomicInteger next = new AtomicInteger();
        Command sequence = Commands.sequence(
                DrivetrainCommands.followSettledWithTimeout(rig.follower, FollowerTestRig::path, 0),
                Commands.runOnce(next::incrementAndGet));
        assertThrows(IllegalStateException.class, sequence::execute);
        assertEquals(0, next.get());
        assertTrue(rig.follower.idle());
    }

    @Test public void strictPathHandlesConfiguredNoHoldEnd() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.holdEnd.set(false);
        Command path = DrivetrainCommands.followSettledWithTimeout(
                rig.follower, FollowerTestRig::path, 1000);
        assertTrue(path.execute().isRunning());
        rig.finishSegment = true;
        rig.follower.update(.02);
        rig.follower.update(.02);
        assertTrue(rig.follower.idle());
        assertTrue(path.execute().isRunning());
        assertTrue(rig.follower.holding());
        rig.pose = new Pose(10, 0, 0);
        rig.follower.update(.02);
        assertFalse(path.execute().isRunning());
    }

    @Test public void ordinaryFollowPreservesConfiguredNoHoldEnd() {
        FollowerTestRig rig = new FollowerTestRig();
        rig.follower.holdEnd.set(false);
        Command path = DrivetrainCommands.follow(rig.follower, FollowerTestRig::path);
        assertTrue(path.execute().isRunning());
        rig.finishSegment = true;
        rig.follower.update(.02);
        assertFalse(path.execute().isRunning());
        rig.follower.update(.02);
        assertTrue(rig.follower.idle());
    }

    @Test public void cancellationStopsTurnAndFollowWithoutAnotherFollowerUpdate() {
        FollowerTestRig rig = new FollowerTestRig();
        Command turn = DrivetrainCommands.turnToHeading(rig.follower, 1);
        turn.execute();
        turn.invalidate();
        assertEquals(1, rig.stops);
        Command path = DrivetrainCommands.follow(rig.follower, FollowerTestRig::path);
        path.execute();
        path.invalidate();
        assertEquals(2, rig.stops);
    }

    @Test public void invalidThresholdsDurationsAndTolerancesFailExplicitly() {
        FollowerTestRig rig = new FollowerTestRig();
        assertThrows(IllegalArgumentException.class,
                () -> DrivetrainCommands.runAtCompletion(rig.follower, Double.NaN, () -> {}));
        assertThrows(IllegalArgumentException.class,
                () -> DrivetrainCommands.followWithTimeout(rig.follower, FollowerTestRig::path, -1));
        assertThrows(IllegalArgumentException.class,
                () -> DrivetrainCommands.turnToHeadingWithTimeout(rig.follower, 0, Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> DrivetrainCommands.followSettledWithTimeout(
                        rig.follower, FollowerTestRig::path, 1000, -1, .1, .1, .1));
    }
}
