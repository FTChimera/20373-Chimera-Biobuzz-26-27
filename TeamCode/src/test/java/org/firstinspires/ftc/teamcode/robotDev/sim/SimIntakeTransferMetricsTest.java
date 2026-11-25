package org.firstinspires.ftc.teamcode.robotDev.sim;

import org.junit.Test;
import static org.junit.Assert.*;

public class SimIntakeTransferMetricsTest {
    private IntakeSequenceMetrics metrics() { return new IntakeSequenceMetrics(1, 0.5, 0.2, 0.6); }

    private void tick(IntakeSequenceMetrics m, double time, boolean motion) {
        IntakeSequenceMetrics.Phase p = IntakeSequenceMetrics.phaseAt(time);
        m.sample(time, m.expectedIntake(p), m.expectedTransfer(p), m.expectedServo(p),
                motion && p.intakeOn() ? -5 : 0, motion && p == IntakeSequenceMetrics.Phase.TRANSFER ? -5 : 0, 0, 0);
    }

    @Test public void boundariesAreHalfOpenAndCompletionIsNotEarly() {
        assertEquals(IntakeSequenceMetrics.Phase.STOP_STOW, IntakeSequenceMetrics.phaseAt(0));
        assertEquals(IntakeSequenceMetrics.Phase.STOP_STOW, IntakeSequenceMetrics.phaseAt(0.499));
        assertEquals(IntakeSequenceMetrics.Phase.DEPLOY_ONLY, IntakeSequenceMetrics.phaseAt(0.5));
        assertEquals(IntakeSequenceMetrics.Phase.INTAKE, IntakeSequenceMetrics.phaseAt(1));
        assertEquals(IntakeSequenceMetrics.Phase.TRANSFER, IntakeSequenceMetrics.phaseAt(3));
        assertEquals(IntakeSequenceMetrics.Phase.FINAL_STOP, IntakeSequenceMetrics.phaseAt(5));
        IntakeSequenceMetrics m = metrics();
        for (int i = 0; i < 550; i++) { tick(m, i / 100.0, true); assertFalse(m.passed); }
        tick(m, 5.5, true);
        assertNull(m.failure);
        assertTrue(m.passed);
        for (IntakeSequenceMetrics.Phase p : IntakeSequenceMetrics.Phase.values()) {
            assertTrue(m.samples[p.ordinal()] >= p.minimumSamples());
            assertTrue(m.span(p) >= p.minimumSpan());
        }
    }

    @Test public void inclusiveToleranceAndInvalidMatching() {
        for (double expected : new double[]{0, 0.2, 0.5, 0.6, 1}) {
            assertTrue(IntakeSequenceMetrics.matches(expected + 0.005, expected, 0.005));
            assertTrue(IntakeSequenceMetrics.matches(expected - 0.005, expected, 0.005));
            assertFalse(IntakeSequenceMetrics.matches(expected + 0.00501, expected, 0.005));
        }
        assertFalse(IntakeSequenceMetrics.matches(Double.NaN, 0, 0.005));
        assertFalse(IntakeSequenceMetrics.matches(0, Double.POSITIVE_INFINITY, 0.005));
        assertFalse(IntakeSequenceMetrics.matches(0, 0, -1));
    }

    @Test public void graceIsBoundedAndMismatchLatches() {
        IntakeSequenceMetrics m = metrics();
        m.sample(0, 1, 1, 0.2, 0, 0, 0, 0);
        m.sample(0.1, 1, 1, 0.2, 0, 0, 0, 0);
        assertNull(m.failure);
        m.sample(0.15, 1, 0, 0.6, 0, 0, 0, 0);
        assertNotNull(m.failure);
        String failure = m.failure;
        for (int i = 16; i <= 550; i++) tick(m, i / 100.0, true);
        assertEquals(failure, m.failure);
        assertFalse(m.passed);
    }

    @Test public void eachAppliedOutputIsChecked() {
        for (int output = 0; output < 3; output++) {
            IntakeSequenceMetrics m = metrics();
            tick(m, 0, false); tick(m, 0.1, false);
            m.sample(0.15, output == 0 ? 0.006 : 0, output == 1 ? 0.006 : 0,
                    output == 2 ? 0.606 : 0.6, 0, 0, 0, 0);
            assertNotNull(m.failure);
        }
    }

    @Test public void activeCommandsAndDeployedServoMustBeApplied() {
        for (int output = 0; output < 3; output++) {
            IntakeSequenceMetrics m = metrics();
            for (int i = 0; i < 315; i++) tick(m, i / 100.0, true);
            m.sample(3.15, output == 0 ? 0 : 1, output == 1 ? 0 : 0.5,
                    output == 2 ? 0.6 : 0.2, -5, -5, 0, 0);
            assertNotNull(m.failure);
            assertFalse(m.passed);
        }
    }

    @Test public void completionTickStillRequiresObservedStop() {
        IntakeSequenceMetrics m = metrics();
        for (int i = 0; i < 550; i++) tick(m, i / 100.0, true);
        m.sample(5.5, 1, 0.5, 0.2, -5, -5, 0, 0);
        assertNotNull(m.failure);
        assertFalse(m.passed);
    }

    @Test public void invalidObservationsFailEvenDuringGrace() {
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            for (int field = 0; field < 8; field++) {
                IntakeSequenceMetrics m = metrics();
                double[] values = {0, 0, 0, 0.6, 0, 0, 0, 0};
                values[field] = bad;
                m.sample(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
                assertNotNull(m.failure);
            }
        }
    }

    @Test public void omittedPhasesLateTicksAndBackwardTimeFail() {
        IntakeSequenceMetrics m = metrics();
        assertFalse(m.passed);
        tick(m, 1, true);
        assertNotNull(m.failure);
        m = metrics(); tick(m, 0, false); tick(m, 0.101, false);
        assertNotNull(m.failure);
        m = metrics(); tick(m, 0.05, false); tick(m, 0.04, false);
        assertNotNull(m.failure);
        m = metrics(); tick(m, 0, false); tick(m, 0, false);
        assertNotNull(m.failure);
        m = metrics();
        m.sample(-0.01, 0, 0, 0.6, 0, 0, 0, 0);
        assertNotNull(m.failure);
    }

    @Test public void undersampledShortAndLongPhasesCannotPass() {
        IntakeSequenceMetrics m = metrics();
        for (int i = 0; i <= 5; i++) tick(m, i / 10.0, true);
        assertNotNull(m.failure); // Only .2-.4: three checked samples.
        m = metrics();
        for (int i = 0; i <= 100; i++) tick(m, i / 100.0, true);
        for (int i = 110; i <= 300; i += 10) tick(m, i / 100.0, true);
        assertNotNull(m.failure); // 2 s phase sampled .1 s apart: span 1.7 < 1.75.
        assertFalse(m.passed);
    }

    @Test public void bothMotorsNeedMeasuredMotionNotOnlyCommandedPower() {
        for (int missing = 0; missing < 2; missing++) {
            IntakeSequenceMetrics m = metrics();
            for (int i = 0; i <= 550; i++) {
                double t = i / 100.0;
                IntakeSequenceMetrics.Phase p = IntakeSequenceMetrics.phaseAt(t);
                m.sample(t, m.expectedIntake(p), m.expectedTransfer(p), m.expectedServo(p),
                        missing == 0 ? 0 : -5, missing == 1 ? 0 : -5, 0, 0);
            }
            assertNotNull(m.failure);
            assertFalse(m.passed);
        }
    }

    @Test public void reverseEncoderMovementCanPassWithoutVelocity() {
        IntakeSequenceMetrics m = metrics();
        for (int i = 0; i <= 550; i++) {
            double t = i / 100.0;
            IntakeSequenceMetrics.Phase p = IntakeSequenceMetrics.phaseAt(t);
            m.sample(t, m.expectedIntake(p), m.expectedTransfer(p), m.expectedServo(p),
                    0, 0, p.intakeOn() ? -i : 0, p == IntakeSequenceMetrics.Phase.TRANSFER ? -i : 0);
        }
        assertTrue(m.passed);
    }

    @Test(expected = IllegalArgumentException.class) public void invalidConstantsAreNotClipped() {
        new IntakeSequenceMetrics(1, 0.5, -0.1, 0.6);
    }
}
