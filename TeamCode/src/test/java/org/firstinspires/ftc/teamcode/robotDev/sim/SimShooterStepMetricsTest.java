package org.firstinspires.ftc.teamcode.robotDev.sim;

import org.junit.Test;
import static org.junit.Assert.*;

public class SimShooterStepMetricsTest {
    @Test public void firstEntryIsNotSustainedSettleAndExcursionResetsDwell() {
        ShooterAimMetrics.Dwell dwell = new ShooterAimMetrics.Dwell();
        assertFalse(dwell.sample(0, false, 0.3));
        assertFalse(dwell.sample(1, true, 0.3));
        assertEquals(1, dwell.firstEntry, 0);
        assertTrue(Double.isNaN(dwell.settledAt));
        assertFalse(dwell.sample(1.09, true, 0.3));
        assertFalse(dwell.sample(1.18, true, 0.3));
        assertFalse(dwell.sample(1.25, false, 0.3));
        assertEquals(0, dwell.current, 0);
        assertFalse(dwell.sample(2, true, 0.3));
        assertFalse(dwell.sample(2.09, true, 0.3));
        assertFalse(dwell.sample(2.18, true, 0.3));
        assertFalse(dwell.sample(2.27, true, 0.3));
        assertTrue(dwell.sample(2.36, true, 0.3));
        assertEquals(2.36, dwell.settledAt, 0);
        assertEquals(1, dwell.firstEntry, 0);
    }

    @Test public void nonfiniteTimeAndWheelReadingsResetContiguousSettling() {
        ShooterAimMetrics.Dwell dwell = new ShooterAimMetrics.Dwell();
        dwell.sample(0, true, 0.3);
        assertFalse(dwell.sample(Double.NaN, true, 0.3));
        assertFalse(dwell.sample(0.4, true, 0.3));
        assertFalse(dwell.sample(0.6, ShooterAimMetrics.wheelsReady(800, 800, 880, Double.NaN, 40), 0.3));
        assertFalse(dwell.sample(0.8, true, 0.3));
        assertFalse(dwell.sample(0.89, true, 0.3));
        assertFalse(dwell.sample(0.98, true, 0.3));
        assertFalse(dwell.sample(1.07, true, 0.3));
        assertTrue(dwell.sample(1.16, true, 0.3));
    }

    @Test public void backwardClockAndNoSamplesCannotSettle() {
        ShooterAimMetrics.Dwell dwell = new ShooterAimMetrics.Dwell();
        assertTrue(Double.isNaN(dwell.firstEntry));
        assertTrue(Double.isNaN(dwell.settledAt));
        dwell.sample(2, true, 0.3);
        assertFalse(dwell.sample(1, true, 0.3));
        assertFalse(dwell.sample(1.3, true, 0.3));
        assertTrue(Double.isNaN(dwell.settledAt));
    }

    @Test public void simulatorPauseDoesNotCountAsContinuousSettling() {
        ShooterAimMetrics.Dwell dwell = new ShooterAimMetrics.Dwell();
        assertFalse(dwell.sample(1, true, 0.3));
        assertFalse(dwell.sample(10, true, 0.3));
        assertEquals(0, dwell.current, 0);
        assertTrue(Double.isNaN(dwell.settledAt));
    }
}
