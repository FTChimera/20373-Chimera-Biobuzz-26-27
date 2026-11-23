package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class VisionFreshnessTest {
    private static final long NOW = 1_000_000_000L;
    private static final double MAX_AGE_MS = 300;
    private static final int PIPELINE = 2;

    private Vision.FrameFreshness frame(double captureMs, double targetingMs, double parseMs) {
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        frame.observe(true, PIPELINE, 123, 0, captureMs, targetingMs, parseMs, NOW);
        return frame;
    }

    @Test
    public void rejectsMissingAndInvalidFrames() {
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        frame.observe(false, PIPELINE, 123, 0, 0, 0, 0, NOW);
        assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        frame.observe(true, PIPELINE, 124, 0, 0, 0, 0, NOW);
        assertTrue(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        frame.invalidate();
        assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        assertFalse(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, NOW - 10_000_000));
    }

    @Test
    public void rejectsWrongPipelineIncludingChangedDesiredPipeline() {
        Vision.FrameFreshness frame = frame(0, 0, 0);
        assertTrue(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        assertFalse(frame.isFresh(PIPELINE + 1, NOW, MAX_AGE_MS));
        assertFalse(frame.isFreshAfter(PIPELINE + 1, NOW, MAX_AGE_MS, NOW - 10_000_000));
    }

    @Test
    public void freshnessIncludesAllLatenciesAndWallClockGuard() {
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        frame.observe(true, PIPELINE, 123, 250, 20, 20, 9, NOW);
        assertTrue(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        assertFalse(frame.isFresh(PIPELINE, NOW + 1, MAX_AGE_MS));
        frame.observe(true, PIPELINE, 124, 301, 0, 0, 0, NOW);
        assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        assertFalse(frame(100, 100, 100).isFresh(PIPELINE, NOW, MAX_AGE_MS));
    }

    @Test
    public void captureRatherThanReceiptMustBeStrictlyAfterBoundary() {
        Vision.FrameFreshness frame = frame(10, 20, 5);
        long estimatedCapture = NOW - 36_000_000;
        assertFalse(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, NOW - 1_000_000));
        assertFalse(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, estimatedCapture));
        assertTrue(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, estimatedCapture - 1));
        assertFalse(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, NOW + 1));
    }

    @Test
    public void boundaryAccountsForStalenessBeforeFirstObservation() {
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        frame.observe(true, PIPELINE, 123, 100, 10, 20, 5, NOW);
        assertFalse(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, NOW - 100_000_000));
        assertTrue(frame.isFreshAfter(PIPELINE, NOW, MAX_AGE_MS, NOW - 137_000_000));
    }

    @Test
    public void cachedAndReparsedFramesNeverGetNewCaptureTimes() {
        Vision.FrameFreshness frame = frame(10, 20, 5);
        long later = NOW + 200_000_000;
        frame.observe(true, PIPELINE, 123, 0, 0, 0, 0, later);
        assertFalse(frame.isFreshAfter(PIPELINE, later, MAX_AGE_MS, NOW));
        assertFalse(frame.isFresh(PIPELINE, NOW + 300_000_000, MAX_AGE_MS));
        frame.observe(true, PIPELINE, 124, 0, 10, 20, 5, later);
        assertTrue(frame.isFreshAfter(PIPELINE, later, MAX_AGE_MS, NOW));
    }

    @Test
    public void missingFrameDoesNotRejuvenateSameTimestampWhenItReturns() {
        Vision.FrameFreshness frame = frame(0, 0, 0);
        frame.invalidate();
        frame.observe(true, PIPELINE, 123, 0, 0, 0, 0, NOW + 400_000_000);
        assertFalse(frame.isFresh(PIPELINE, NOW + 400_000_000, MAX_AGE_MS));
    }

    @Test
    public void olderTimingOnDuplicateCanOnlyMoveCaptureEstimateBackward() {
        Vision.FrameFreshness frame = frame(0, 0, 0);
        frame.observe(true, PIPELINE, 123, 100, 0, 0, 0, NOW + 10_000_000);
        assertFalse(frame.isFreshAfter(PIPELINE, NOW + 10_000_000,
                MAX_AGE_MS, NOW - 50_000_000));
        frame.observe(true, PIPELINE, 123, 0, 0, 0, 0, NOW + 20_000_000);
        assertFalse(frame.isFreshAfter(PIPELINE, NOW + 20_000_000,
                MAX_AGE_MS, NOW - 50_000_000));
        frame.observe(true, PIPELINE, 123, 301, 0, 0, 0, NOW + 30_000_000);
        assertFalse(frame.isFresh(PIPELINE, NOW + 30_000_000, MAX_AGE_MS));
    }

    @Test
    public void rejectsMalformedTimingAndFreshnessLimits() {
        double[] invalid = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, -1};
        for (double value : invalid) {
            assertFalse(frame(value, 0, 0).isFresh(PIPELINE, NOW, MAX_AGE_MS));
            assertFalse(frame(0, value, 0).isFresh(PIPELINE, NOW, MAX_AGE_MS));
            assertFalse(frame(0, 0, value).isFresh(PIPELINE, NOW, MAX_AGE_MS));
            assertFalse(frame(0, 0, 0).isFresh(PIPELINE, NOW, value));
        }
        assertFalse(frame(Double.MAX_VALUE, Double.MAX_VALUE, 0)
                .isFresh(PIPELINE, NOW, MAX_AGE_MS));
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        frame.observe(true, PIPELINE, 123, -1, 0, 0, 0, NOW);
        assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        for (double timestamp : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            frame.observe(true, PIPELINE, timestamp, 0, 0, 0, 0, NOW);
            assertFalse(frame.isFresh(PIPELINE, NOW, MAX_AGE_MS));
        }
    }

    @Test
    public void monotonicAgeWorksWithNegativeNanoTimeAndWraparound() {
        Vision.FrameFreshness frame = new Vision.FrameFreshness();
        frame.observe(true, PIPELINE, 123, 0, 0, 0, 0, -10_000_000);
        assertTrue(frame.isFreshAfter(PIPELINE, -9_000_000, MAX_AGE_MS, -12_000_000));
        frame.observe(true, PIPELINE, 124, 0, 0, 0, 0, Long.MAX_VALUE - 500_000);
        assertTrue(frame.isFresh(PIPELINE, Long.MIN_VALUE + 500_000, MAX_AGE_MS));
        assertFalse(frame.isFreshAfter(PIPELINE, Long.MIN_VALUE + 500_000,
                MAX_AGE_MS, Long.MAX_VALUE));
        assertFalse(frame.isFresh(PIPELINE, Long.MAX_VALUE - 600_000, MAX_AGE_MS));
    }
}
