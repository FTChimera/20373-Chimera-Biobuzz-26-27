package org.firstinspires.ftc.teamcode.robotDev.sim;

import org.junit.Test;

import static org.junit.Assert.*;

public class DriveDiagnosticProgressTest {
    private static final double[] STILL = {0, 0, 0, 0};
    private static final double[] MOVING = {30, -30, 30, -30};
    private static final double[] POWER = {.2, .2, .2, .2};

    private DriveDiagnosticProgress baseline() {
        DriveDiagnosticProgress p = new DriveDiagnosticProgress(20);
        assertEquals("RUNNING", sample(p, 0, 9, 9, STILL, false, false, false));
        return p;
    }

    private String sample(DriveDiagnosticProgress p, double time, double x, double rawX,
                          double[] ticks, boolean started, boolean done, boolean destination) {
        return p.sample(time, x, 25, 0, rawX, 25, 0, ticks, POWER, done ? 1 : 0,
                started, done, destination, false);
    }

    @Test public void passRequiresStartedCompletedPathLiveTranslationAndWheels() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("RUNNING", sample(p, .5, 15, 15, MOVING, true, false, true));
        assertEquals("PASS", sample(p, 1, 40, 40, MOVING, true, true, true));
        assertEquals(31, p.poseMovement, 0);
        assertEquals(31, p.pinpointMovement, 0);
        assertEquals(30, p.wheelMovement, 0);
    }

    @Test public void wheelsMovingButFixedPoseFailsWithStopOtherDiagnosticsMessage() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("RUNNING", sample(p, 1.99, 9, 9, MOVING, true, false, false));
        assertEquals("FAIL: Pinpoint dead; stop other path diagnostics",
                sample(p, 2, 9, 9, MOVING, true, false, false));
    }

    @Test public void filteredPoseDriftDoesNotHideDeadRawPinpoint() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: Pinpoint dead; stop other path diagnostics",
                sample(p, 2, 40, 9, MOVING, true, false, true));
    }

    @Test public void rawPinpointDyingMidRouteHasBoundedRollingWindow() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("RUNNING", sample(p, 1, 15, 15, MOVING, true, false, false));
        assertEquals("RUNNING", sample(p, 2.99, 15, 15, new double[]{60, 60, 60, 60}, true, false, false));
        assertEquals("FAIL: Pinpoint dead; stop other path diagnostics",
                sample(p, 3, 15, 15, new double[]{60, 60, 60, 60}, true, false, false));
    }

    @Test public void controllerCompletionOutsideEndpointOrParkFails() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: follower completed outside destination",
                sample(p, 1, 12, 12, MOVING, true, true, false));
    }

    @Test public void earlyFollowerExitInsideParkZoneCannotPass() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: follower exited before full route completion",
                p.sample(1, 40, 25, 0, 40, 25, 0, MOVING, POWER,
                        0.8, true, true, true, false));
    }

    @Test public void initialSetPoseIsNotEvidenceOfWorkingPinpoint() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: completion without live translational Pinpoint movement",
                sample(p, 1, 9, 9, MOVING, true, true, true));
    }

    @Test public void completionBeforeFollowCannotPass() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: completion without a started path",
                sample(p, 1, 40, 40, MOVING, false, true, true));
    }

    @Test public void poseJumpWithoutIndependentDriveMovementCannotPass() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: completion without independently observed drive movement",
                sample(p, 1, 40, 40, STILL, true, true, true));
    }

    @Test public void fixedWheelsAreDrivetrainFailureNotDeadPinpoint() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("RUNNING", sample(p, 2, 9, 9, STILL, true, false, false));
        assertEquals("FAIL: timeout; no independently observed drivetrain movement",
                sample(p, 20, 9, 9, STILL, true, false, false));
    }

    @Test public void movingIncompletePathTimesOutEvenWhenDestinationIsTrue() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: timeout; path did not complete",
                sample(p, 20, 40, 40, MOVING, true, false, true));
    }

    @Test public void timeoutWinsOverLateCompletion() {
        DriveDiagnosticProgress p = baseline();
        assertTrue(sample(p, 20, 40, 40, MOVING, true, true, true).startsWith("FAIL: timeout"));
    }

    @Test public void delayedFirstLiveBaselineDoesNotExtendEntireTestDeadline() {
        DriveDiagnosticProgress p = new DriveDiagnosticProgress(20);
        assertEquals("FAIL: timeout; no independently observed drivetrain movement",
                sample(p, 20, 9, 9, STILL, false, false, false));
    }

    @Test public void cancellationIsTerminalAndCannotBecomePass() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: cancelled/STOP", p.cancel());
        assertEquals("FAIL: cancelled/STOP", sample(p, 1, 40, 40, MOVING, true, true, true));
        assertTrue(p.finished());
    }

    @Test public void stopPreservesFinalSuccessfulNumbers() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("PASS", sample(p, 1, 40, 40, MOVING, true, true, true));
        assertEquals("PASS", p.cancel());
        assertEquals(31, p.pinpointMovement, 0);
    }

    @Test public void cancelledSampleFailsEvenAtDestination() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: cancelled/STOP", p.sample(1, 40, 25, 0, 40, 25, 0,
                MOVING, POWER, 1, true, true, true, true));
    }

    @Test public void nonfinitePoseRawPoseHeadingClockAndCompletionFail() {
        for (int index = 0; index < 8; index++) {
            for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
                DriveDiagnosticProgress p = baseline();
                double[] v = {1, 40, 25, 0, 40, 25, 0, 1};
                v[index] = invalid;
                assertTrue(p.sample(v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                        MOVING, POWER, v[7], true, true, true, false).startsWith("FAIL: nonfinite"));
            }
        }
    }

    @Test public void invalidWheelSamplesAndPowerCannotPass() {
        for (double[] invalid : new double[][]{null, {}, {1, 2, 3}, {1, 2, 3, Double.NaN}}) {
            assertTrue(sample(baseline(), 1, 40, 40, invalid, true, true, true).startsWith("FAIL: nonfinite"));
            assertTrue(baseline().sample(1, 40, 25, 0, 40, 25, 0, MOVING,
                    invalid, 1, true, true, true, false).startsWith("FAIL: nonfinite"));
        }
    }

    @Test public void headingWrapAndExplicitEndpointThresholds() {
        assertEquals(Math.toRadians(2), DriveDiagnosticProgress.headingError(
                Math.toRadians(179), Math.toRadians(-179)), 1e-12);
        assertTrue(DriveDiagnosticProgress.atEndpoint(40, 10, Math.toRadians(359), 40, 10, 0));
        assertTrue(DriveDiagnosticProgress.atEndpoint(38, 10, 0, 40, 10, 0));
        assertFalse(DriveDiagnosticProgress.atEndpoint(37.99, 10, 0, 40, 10, 0));
        assertFalse(DriveDiagnosticProgress.atEndpoint(40, 10, Math.toRadians(9), 40, 10, 0));
        assertFalse(DriveDiagnosticProgress.atEndpoint(40, 10, Double.NaN, 40, 10, 0));
        assertFalse(DriveDiagnosticProgress.atEndpoint(40, 10, 0, Double.POSITIVE_INFINITY, 10, 0));
    }

    @Test public void backwardClockFailsAndTimeoutConfigurationMustBeValid() {
        DriveDiagnosticProgress p = baseline();
        assertEquals("FAIL: clock moved backwards", sample(p, -1, 9, 9, STILL, false, false, false));
        for (double bad : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            try {
                new DriveDiagnosticProgress(bad);
                fail("invalid timeout accepted");
            } catch (IllegalArgumentException expected) {
                assertNotNull(expected.getMessage());
            }
        }
    }
}
