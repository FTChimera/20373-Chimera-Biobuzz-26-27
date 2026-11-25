package org.firstinspires.ftc.teamcode.robotDev.sim;

import org.junit.Test;
import static org.junit.Assert.*;

public class SimHoldJitterMetricsTest {
    @Test public void headingErrorAndCircularJitterWrapAcross180Degrees() {
        assertEquals(2, Math.toDegrees(ShooterAimMetrics.wrappedError(Math.toRadians(-179), Math.toRadians(179))), 1e-10);
        ShooterAimMetrics.Heading samples = new ShooterAimMetrics.Heading();
        samples.sample(Math.toRadians(179), 0);
        samples.sample(Math.toRadians(-179), 0);
        assertEquals(2, Math.toDegrees(samples.peakToPeak()), 1e-10);
        assertFalse(samples.within(3));
    }

    @Test public void emptyAndNonfiniteSamplesFail() {
        ShooterAimMetrics.Heading samples = new ShooterAimMetrics.Heading();
        assertTrue(Double.isNaN(samples.rms()));
        assertTrue(Double.isNaN(samples.peakToPeak()));
        assertFalse(samples.within(3));
        samples.sample(0, 0);
        samples.sample(Double.NaN, 0);
        assertFalse(samples.within(3));
        assertTrue(Double.isNaN(samples.peakToPeak()));
    }

    @Test public void fullStationaryDurationAndEveryErrorAreRequired() {
        ShooterAimMetrics.Heading samples = new ShooterAimMetrics.Heading();
        assertFalse(ShooterAimMetrics.holdPass(samples, 5, 5, 0, 0, 0.5, 0.5, 3));
        samples.sample(Math.nextDown(Math.toRadians(3)), 0);
        assertTrue(ShooterAimMetrics.holdPass(samples, 5, 5, 0.5, 0.5, 0.5, 0.5, 3));
        assertFalse(ShooterAimMetrics.holdPass(samples, 4.999, 5, 0, 0, 0.5, 0.5, 3));
        assertFalse(ShooterAimMetrics.holdPass(samples, 5, 5, 0.501, 0, 0.5, 0.5, 3));
        assertFalse(ShooterAimMetrics.holdPass(samples, 5, 5, 0, 0.501, 0.5, 0.5, 3));
        assertFalse(ShooterAimMetrics.holdPass(samples, 5, 5, 0, Double.NaN, 0.5, 0.5, 3));
        samples.sample(Math.toRadians(3.001), 0);
        assertFalse(ShooterAimMetrics.holdPass(samples, 5, 5, 0, 0, 0.5, 0.5, 3));
    }

    @Test public void singleIdenticalSampleHasZeroJitter() {
        ShooterAimMetrics.Heading samples = new ShooterAimMetrics.Heading();
        samples.sample(1, 1);
        assertEquals(0, samples.peakToPeak(), 0);
        samples.sample(1, 1);
        assertEquals(0, samples.peakToPeak(), 0);
    }

    @Test public void fiveSecondWindowNeedsSamplingCoverageNotJustWallTime() {
        assertTrue(ShooterAimMetrics.windowCovered(4.99, 5.01, 0.02, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(0, 5, 5, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(4.99, 4.999, 0.02, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(4.8, 5, 0.02, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(4.99, 5.2, 0.02, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(4.99, 5.01, 0.101, 5, 0.1));
        assertFalse(ShooterAimMetrics.windowCovered(Double.NaN, 5, 0, 5, 0.1));
    }
}
