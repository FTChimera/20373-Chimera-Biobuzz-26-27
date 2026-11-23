package org.firstinspires.ftc.teamcode.planners.commands;

import static org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands.ShotOutcome.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShooterCommandsTest {
    private final ShooterCommands.ShotLifecycle shot = new ShooterCommands.ShotLifecycle();

    @Test
    public void startsNotAttemptedAndWaitsForSpeed() {
        assertEquals(NOT_STARTED, shot.outcome());
        assertFalse(shot.attempted());
        shot.start();
        assertEquals(WAITING_FOR_SPEED, shot.outcome());
        assertFalse(shot.attempted());
    }

    @Test
    public void confirmedSpinupBecomesAttemptOnlyAfterTimedFeedCompletes() {
        shot.start();
        shot.feed(false);
        assertEquals(FEEDING, shot.outcome());
        assertFalse(shot.attempted());
        shot.end(false);
        assertEquals(ATTEMPTED, shot.outcome());
        assertTrue(shot.attempted());
    }

    @Test
    public void spinupTimeoutStillFeedsAndPersistsInCompletedOutcome() {
        shot.start();
        shot.feed(true);
        assertEquals(FEEDING, shot.outcome());
        assertFalse(shot.attempted());
        shot.end(false);
        assertEquals(ATTEMPTED_SPINUP_TIMEOUT, shot.outcome());
        assertTrue(shot.attempted());
        shot.end(false);
        assertEquals(ATTEMPTED_SPINUP_TIMEOUT, shot.outcome());
    }

    @Test
    public void interruptionWhileWaitingDoesNotCountAsAttempt() {
        shot.start();
        shot.end(true);
        assertEquals(INTERRUPTED, shot.outcome());
        assertFalse(shot.attempted());
    }

    @Test
    public void interruptionDuringEitherKindOfFeedDoesNotCountAsFullBatch() {
        for (boolean timedOut : new boolean[]{false, true}) {
            shot.start();
            shot.feed(timedOut);
            shot.end(true);
            assertEquals(INTERRUPTED, shot.outcome());
            assertFalse(shot.attempted());
        }
    }

    @Test
    public void endingWithoutFeedingCannotClaimAnAttempt() {
        shot.end(false);
        assertEquals(INTERRUPTED, shot.outcome());
        shot.start();
        shot.end(false);
        assertEquals(INTERRUPTED, shot.outcome());
        assertFalse(shot.attempted());
    }

    @Test
    public void repeatedEndPreservesTerminalOutcomes() {
        shot.start();
        shot.feed(false);
        shot.end(false);
        shot.end(true);
        assertEquals(ATTEMPTED, shot.outcome());
        shot.start();
        shot.feed(true);
        shot.end(true);
        shot.end(false);
        assertEquals(INTERRUPTED, shot.outcome());
    }

    @Test
    public void restartClearsPreviousTimeoutAndAttempt() {
        shot.start();
        shot.feed(true);
        shot.end(false);
        shot.start();
        assertEquals(WAITING_FOR_SPEED, shot.outcome());
        assertFalse(shot.attempted());
        shot.feed(false);
        shot.end(false);
        assertEquals(ATTEMPTED, shot.outcome());
    }

    @Test(expected = IllegalStateException.class)
    public void cannotFeedBeforeStarting() {
        shot.feed(false);
    }

    @Test(expected = IllegalStateException.class)
    public void cannotFeedTwiceInOneRun() {
        shot.start();
        shot.feed(true);
        shot.feed(false);
    }
}
