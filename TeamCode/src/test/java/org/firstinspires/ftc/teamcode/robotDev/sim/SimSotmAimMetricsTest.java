package org.firstinspires.ftc.teamcode.robotDev.sim;

import org.junit.Test;
import static org.junit.Assert.*;

public class SimSotmAimMetricsTest {
    @Test public void gatesAreInclusiveAndRequireBothPositiveWheelTargets() {
        assertTrue(ShooterAimMetrics.wheelsReady(800, 760, 880, 920, 40));
        assertFalse(ShooterAimMetrics.wheelsReady(800, 759.99, 880, 880, 40));
        assertFalse(ShooterAimMetrics.wheelsReady(800, 800, 880, 920.01, 40));
        assertFalse(ShooterAimMetrics.wheelsReady(0, 0, 880, 880, 40));
        assertFalse(ShooterAimMetrics.wheelsReady(800, 800, 0, 0, 40));
        assertTrue(ShooterAimMetrics.withinHeadingError(Math.toRadians(3), Math.toRadians(3)));
        assertFalse(ShooterAimMetrics.withinHeadingError(Math.nextUp(Math.toRadians(3)), Math.toRadians(3)));
        // Production atan2(sin, cos) may add an ULP; do not add a diagnostic-only epsilon.
        assertTrue(ShooterAimMetrics.headingReady(Math.nextDown(Math.toRadians(3)), 0, 3));
        double exactWrappedBoundary = ShooterAimMetrics.wrappedError(Math.toRadians(3), 0);
        assertTrue(ShooterAimMetrics.headingReady(Math.toRadians(3), 0, Math.toDegrees(exactWrappedBoundary)));
        assertFalse(ShooterAimMetrics.headingReady(Math.toRadians(3.001), 0, 3));
        assertFalse(ShooterAimMetrics.yAutoFire(false, true, true));
        assertFalse(ShooterAimMetrics.yAutoFire(true, false, true));
        assertFalse(ShooterAimMetrics.yAutoFire(true, true, false));
        assertTrue(ShooterAimMetrics.yAutoFire(true, true, true));
    }

    @Test public void invalidMetricsNeverOpenGates() {
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertFalse(ShooterAimMetrics.wheelsReady(800, invalid, 880, 880, 40));
            assertFalse(ShooterAimMetrics.wheelsReady(800, 800, 880, invalid, 40));
            assertFalse(ShooterAimMetrics.wheelsReady(invalid, 800, 880, 880, 40));
            assertFalse(ShooterAimMetrics.wheelsReady(800, 800, invalid, 880, 40));
            assertFalse(ShooterAimMetrics.headingReady(invalid, 0, 3));
            assertFalse(ShooterAimMetrics.headingReady(0, invalid, 3));
        }
    }

    @Test public void noSamplesOrNoMeasuredMovementCannotPass() {
        ShooterAimMetrics.Heading moving = new ShooterAimMetrics.Heading();
        assertFalse(ShooterAimMetrics.movingPass(moving, 2, 5, 1, 3));
        for (int i = 0; i < 5; i++) moving.sample(0, 0);
        assertFalse(ShooterAimMetrics.movingPass(moving, 0, 5, 1, 3));
        assertFalse(ShooterAimMetrics.movingPass(moving, Double.NaN, 5, 1, 3));
        assertTrue(ShooterAimMetrics.movingPass(moving, 1, 5, 1, 3));
        moving.sample(Math.toRadians(3.01), 0);
        assertFalse(ShooterAimMetrics.movingPass(moving, 1, 5, 1, 3));
    }
}
