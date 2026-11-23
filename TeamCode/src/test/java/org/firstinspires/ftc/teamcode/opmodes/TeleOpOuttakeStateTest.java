package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.FeedMode.EJECT;
import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.FeedMode.FAST;
import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.FeedMode.SLOW;
import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.FeedMode.STOP;
import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.ShooterMode.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.opmodes.TeleOpOuttakeState.FeedMode;
import org.junit.Test;

public class TeleOpOuttakeStateTest {
    private static final int REVERSE_HELD = 1;
    private static final int PANIC_HELD = 1 << 1;
    private static final int ENABLED = 1 << 2;
    private static final int RT = 1 << 3;
    private static final int LT = 1 << 4;
    private static final int Y = 1 << 5;
    private static final int EJECT_HELD = 1 << 6;
    private static final int SPEED_READY = 1 << 7;
    private static final int HEADING_READY = 1 << 8;
    private static final int ALL_INPUTS = (1 << 9) - 1;

    @Test
    public void initiallyShooterAndFeedAreOff() {
        assertState(new TeleOpOuttakeState(), OFF, STOP);
    }

    @Test
    public void reverseWinsSimultaneousPanicAndAllNormalRequests() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ALL_INPUTS, REVERSE, STOP);
        tick(state, ALL_INPUTS, REVERSE, STOP);
        tick(state, ALL_INPUTS & ~ENABLED & ~Y, REVERSE, STOP);
    }

    @Test
    public void panicOwnsFeedEvenWithShooterDisabledAndEjectHeld() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ALL_INPUTS & ~REVERSE_HELD, PANIC, FeedMode.PANIC);
        tick(state, PANIC_HELD | EJECT_HELD | RT | LT, PANIC, FeedMode.PANIC);
        tick(state, PANIC_HELD, PANIC, FeedMode.PANIC);
    }

    @Test
    public void releasingReverseRecoversHeldPanicThenRestoresSmart() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED | PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, ENABLED | PANIC_HELD | REVERSE_HELD, REVERSE, STOP);
        tick(state, ENABLED | PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, ENABLED, SMART, STOP);
    }

    @Test
    public void releasingPanicCannotCancelHeldReverse() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED | REVERSE_HELD, REVERSE, STOP);
        tick(state, ENABLED | REVERSE_HELD | PANIC_HELD, REVERSE, STOP);
        tick(state, ENABLED | REVERSE_HELD, REVERSE, STOP);
        tick(state, ENABLED, SMART, STOP);
    }

    @Test
    public void simultaneousOverrideReleaseRestoresCurrentEnableState() {
        for (int enabled : new int[] {0, ENABLED}) {
            TeleOpOuttakeState state = new TeleOpOuttakeState();
            tick(state, REVERSE_HELD | PANIC_HELD | ENABLED, REVERSE, STOP);
            tick(state, enabled, enabled == 0 ? OFF : SMART, STOP);
            tick(state, REVERSE_HELD | PANIC_HELD, REVERSE, STOP);
            tick(state, enabled, enabled == 0 ? OFF : SMART, STOP);
        }
    }

    @Test
    public void overrideHandoffInEitherDirectionUsesCurrentHeldState() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, REVERSE_HELD, REVERSE, STOP);
        tick(state, PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, 0, OFF, STOP);
    }

    @Test
    public void manualBurstWaitsForSpeedButSurvivesSpeedAndHeadingLoss() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED | RT, SMART, STOP);
        tick(state, ENABLED | RT, SMART, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
        tick(state, ENABLED | RT, SMART, FAST);
        tick(state, ENABLED | RT | Y, SMART, FAST);
        tick(state, ENABLED | RT | HEADING_READY, SMART, FAST);
    }

    @Test
    public void releasingManualTriggerRequiresSpeedOnNextBurst() {
        TeleOpOuttakeState state = readyManualBurst();
        tick(state, ENABLED, SMART, STOP);
        tick(state, ENABLED | RT, SMART, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
    }

    @Test
    public void explicitShooterDisableStopsAndResetsManualBurst() {
        TeleOpOuttakeState state = readyManualBurst();
        tick(state, RT | SPEED_READY | HEADING_READY, OFF, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, STOP);
        tick(state, ENABLED | RT, SMART, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
    }

    @Test
    public void eitherOverrideResetsManualLatchAndRequiresFreshSpeed() {
        for (int override : new int[] {PANIC_HELD, REVERSE_HELD}) {
            TeleOpOuttakeState state = readyManualBurst();
            tick(state, ENABLED | RT | override,
                    override == PANIC_HELD ? PANIC : REVERSE,
                    override == PANIC_HELD ? FeedMode.PANIC : STOP);
            tick(state, ENABLED | RT | SPEED_READY, SMART, STOP);
            tick(state, ENABLED | RT, SMART, STOP);
            tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
        }
    }

    @Test
    public void ejectResetsManualLatchWithoutLeavingSmart() {
        TeleOpOuttakeState state = readyManualBurst();
        tick(state, ENABLED | RT | EJECT_HELD | SPEED_READY, SMART, EJECT);
        tick(state, ENABLED | RT, SMART, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
    }

    @Test
    public void slowFeedDoesNotRequireShooterSpeedOrHeading() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, LT, OFF, SLOW);
        tick(state, LT | RT | Y, OFF, SLOW);
        tick(state, ENABLED | LT, SMART, SLOW);
        tick(state, ENABLED | LT, SMART, SLOW);
        tick(state, LT, OFF, SLOW);
    }

    @Test
    public void slowBeatsLatchedManualFastButDoesNotResetBurst() {
        TeleOpOuttakeState state = readyManualBurst();
        tick(state, ENABLED | RT | LT, SMART, SLOW);
        tick(state, ENABLED | RT, SMART, FAST);
        tick(state, ENABLED | RT | LT | SPEED_READY, SMART, SLOW);
        tick(state, ENABLED | RT, SMART, FAST);
    }

    @Test
    public void readyAutomaticFeedBeatsSlowAndRechecksBothGatesEachTick() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED, SMART, STOP);
        tick(state, ENABLED | Y | LT | SPEED_READY | HEADING_READY, SMART, FAST);
        tick(state, ENABLED | Y | LT | SPEED_READY, SMART, SLOW);
        tick(state, ENABLED | Y | LT | HEADING_READY, SMART, SLOW);
        tick(state, ENABLED | Y | LT, SMART, SLOW);
        tick(state, ENABLED | Y | LT | SPEED_READY | HEADING_READY, SMART, FAST);
    }

    @Test
    public void automaticFeedRequiresBothGatesAndNeverLatches() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED, SMART, STOP);
        for (int gates : new int[] {
                0, SPEED_READY, HEADING_READY, SPEED_READY | HEADING_READY,
                SPEED_READY, SPEED_READY | HEADING_READY, HEADING_READY, 0}) {
            tick(state, ENABLED | Y | gates, SMART,
                    gates == (SPEED_READY | HEADING_READY) ? FAST : STOP);
        }
        tick(state, ENABLED | SPEED_READY | HEADING_READY, SMART, STOP);
        tick(state, Y | SPEED_READY | HEADING_READY, OFF, STOP);
    }

    @Test
    public void automaticFeedDoesNotPrimeManualBurst() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED, SMART, STOP);
        tick(state, ENABLED | Y | SPEED_READY | HEADING_READY, SMART, FAST);
        tick(state, ENABLED | RT, SMART, STOP);
    }

    @Test
    public void smartEntryBlocksFastForOneTickFromEveryOtherMode() {
        for (int prior : new int[] {0, PANIC_HELD, REVERSE_HELD}) {
            for (int request : new int[] {RT, Y, RT | Y}) {
                TeleOpOuttakeState state = new TeleOpOuttakeState();
                update(state, prior);
                int readyRequest = ENABLED | request | SPEED_READY | HEADING_READY;
                tick(state, readyRequest, SMART, STOP);
                tick(state, readyRequest, SMART, FAST);
            }
        }
    }

    @Test
    public void smartEntryDoesNotBlockIndependentSlowOrEject() {
        for (int prior : new int[] {0, PANIC_HELD, REVERSE_HELD}) {
            TeleOpOuttakeState state = new TeleOpOuttakeState();
            update(state, prior);
            tick(state, ENABLED | RT | Y | LT | SPEED_READY | HEADING_READY, SMART, SLOW);
            update(state, prior);
            tick(state, ENABLED | RT | Y | EJECT_HELD | SPEED_READY | HEADING_READY,
                    SMART, EJECT);
        }
    }

    @Test
    public void ejectBeatsNormalFeedsAndRestoresThemWhenReleased() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, EJECT_HELD | LT, OFF, EJECT);
        tick(state, LT, OFF, SLOW);
        tick(state, ENABLED, SMART, STOP);
        tick(state, ENABLED | EJECT_HELD | RT | Y | LT | SPEED_READY | HEADING_READY,
                SMART, EJECT);
        tick(state, ENABLED | Y | LT | SPEED_READY | HEADING_READY, SMART, FAST);
    }

    @Test
    public void heldEjectReturnsAfterOverridesRelease() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED | EJECT_HELD, SMART, EJECT);
        tick(state, ENABLED | EJECT_HELD | PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, ENABLED | EJECT_HELD | PANIC_HELD | REVERSE_HELD, REVERSE, STOP);
        tick(state, ENABLED | EJECT_HELD | PANIC_HELD, PANIC, FeedMode.PANIC);
        tick(state, ENABLED | EJECT_HELD, SMART, EJECT);
        tick(state, EJECT_HELD, OFF, EJECT);
        tick(state, 0, OFF, STOP);
    }

    @Test
    public void exhaustiveOrderedInputTransitionsRespectArbitrationAndBurstHistory() {
        // Two identical setup ticks allow a ready RT to latch after SMART entry.
        for (int before = 0; before <= ALL_INPUTS; before++) {
            for (int after = 0; after <= ALL_INPUTS; after++) {
                TeleOpOuttakeState state = new TeleOpOuttakeState();
                update(state, before);
                update(state, before);
                update(state, after);

                TeleOpOuttakeState.ShooterMode expectedShooter = shooterFor(after);
                boolean stayedSmart = shooterFor(before) == SMART && expectedShooter == SMART;
                boolean previouslyLatched = shooterFor(before) == SMART
                        && has(before, RT) && has(before, SPEED_READY) && !has(before, EJECT_HELD);
                boolean manualReady = stayedSmart && has(after, RT)
                        && (has(after, SPEED_READY) || previouslyLatched);
                boolean autoReady = stayedSmart && has(after, Y)
                        && has(after, SPEED_READY) && has(after, HEADING_READY);
                TeleOpOuttakeState.FeedMode expectedFeed;
                if (has(after, REVERSE_HELD)) expectedFeed = STOP;
                else if (has(after, PANIC_HELD)) expectedFeed = FeedMode.PANIC;
                else if (has(after, EJECT_HELD)) expectedFeed = EJECT;
                else if (autoReady) expectedFeed = FAST;
                else if (has(after, LT)) expectedFeed = SLOW;
                else if (manualReady) expectedFeed = FAST;
                else expectedFeed = STOP;

                String transition = "before=" + before + ", after=" + after;
                assertEquals(transition, expectedShooter, state.shooter);
                assertEquals(transition, expectedFeed, state.feed);
            }
        }
    }

    @Test
    public void headingReadinessWrapsAcrossBothPiBoundaries() {
        assertTrue(TeleOpOuttakeState.headingReady(Math.toRadians(179), Math.toRadians(-179), 3));
        assertTrue(TeleOpOuttakeState.headingReady(Math.toRadians(-179), Math.toRadians(179), 3));
        assertFalse(TeleOpOuttakeState.headingReady(Math.toRadians(179), Math.toRadians(-179), 1));
        assertFalse(TeleOpOuttakeState.headingReady(Math.toRadians(-179), Math.toRadians(179), 1));
        assertTrue(TeleOpOuttakeState.headingReady(Math.PI, -Math.PI, 0.001));
        assertTrue(TeleOpOuttakeState.headingReady(-Math.PI, Math.PI, 0.001));
        assertTrue(TeleOpOuttakeState.headingReady(4 * Math.PI + 0.1, 0.1, 0.001));
        assertTrue(TeleOpOuttakeState.headingReady(0.1, -4 * Math.PI + 0.1, 0.001));
    }

    @Test
    public void headingToleranceIsInclusiveAndMeasuredInDegrees() {
        assertTrue(TeleOpOuttakeState.headingReady(0, 0, 0));
        assertTrue(TeleOpOuttakeState.headingReady(1.25, 1.25, 0));
        assertFalse(TeleOpOuttakeState.headingReady(0, 0.001, 0));
        for (int sign : new int[] {-1, 1}) {
            assertTrue(TeleOpOuttakeState.headingReady(sign * Math.PI / 2, 0, 90));
            assertTrue(TeleOpOuttakeState.headingReady(0, sign * Math.PI / 2, 90));
            assertFalse(TeleOpOuttakeState.headingReady(sign * (Math.PI / 2 + 1e-8), 0, 90));
            assertTrue(TeleOpOuttakeState.headingReady(sign * (Math.PI / 2 - 1e-8), 0, 90));
            assertTrue(TeleOpOuttakeState.headingReady(sign * Math.PI, 0, 180));
        }
        assertTrue(TeleOpOuttakeState.headingReady(Math.toRadians(4), 0, 5));
        assertFalse(TeleOpOuttakeState.headingReady(Math.toRadians(6), 0, 5));
    }

    @Test
    public void nonfiniteAnglesAreNeverHeadingReady() {
        double[] invalidAngles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double invalid : invalidAngles) {
            for (double tolerance : new double[] {0, 5, 180}) {
                assertFalse(TeleOpOuttakeState.headingReady(invalid, 0, tolerance));
                assertFalse(TeleOpOuttakeState.headingReady(0, invalid, tolerance));
                for (double other : invalidAngles) {
                    assertFalse(TeleOpOuttakeState.headingReady(invalid, other, tolerance));
                }
            }
        }
    }

    @Test
    public void invalidHeadingToleranceThrowsEvenWhenAnglesAreInvalid() {
        double[] invalidTolerances = {
                -Double.MIN_VALUE, -1, Math.nextUp(180.0),
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        };
        for (double tolerance : invalidTolerances) {
            assertThrows(IllegalArgumentException.class,
                    () -> TeleOpOuttakeState.headingReady(0, 0, tolerance));
            assertThrows(IllegalArgumentException.class,
                    () -> TeleOpOuttakeState.headingReady(Double.NaN, Double.NaN, tolerance));
        }
    }

    private static TeleOpOuttakeState readyManualBurst() {
        TeleOpOuttakeState state = new TeleOpOuttakeState();
        tick(state, ENABLED | RT | SPEED_READY, SMART, STOP);
        tick(state, ENABLED | RT | SPEED_READY, SMART, FAST);
        return state;
    }

    private static TeleOpOuttakeState.ShooterMode shooterFor(int inputs) {
        if (has(inputs, REVERSE_HELD)) return REVERSE;
        if (has(inputs, PANIC_HELD)) return PANIC;
        return has(inputs, ENABLED) ? SMART : OFF;
    }

    private static boolean has(int inputs, int flag) {
        return (inputs & flag) != 0;
    }

    private static void update(TeleOpOuttakeState state, int inputs) {
        state.update(has(inputs, REVERSE_HELD), has(inputs, PANIC_HELD), has(inputs, ENABLED),
                has(inputs, RT), has(inputs, LT), has(inputs, Y), has(inputs, EJECT_HELD),
                has(inputs, SPEED_READY), has(inputs, HEADING_READY));
    }

    private static void tick(TeleOpOuttakeState state, int inputs,
                             TeleOpOuttakeState.ShooterMode shooter,
                             TeleOpOuttakeState.FeedMode feed) {
        update(state, inputs);
        assertState(state, shooter, feed);
    }

    private static void assertState(TeleOpOuttakeState state,
                                    TeleOpOuttakeState.ShooterMode shooter,
                                    TeleOpOuttakeState.FeedMode feed) {
        assertEquals(shooter, state.shooter);
        assertEquals(feed, state.feed);
    }
}
