package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Heading check for MainTeleOp auto-fire: wrap-safe, degrees, inclusive tolerance. */
public class MainTeleOpTest {

    @Test
    public void headingReadinessWrapsAcrossBothPiBoundaries() {
        assertTrue(MainTeleOp.headingReady(Math.toRadians(179), Math.toRadians(-179), 3));
        assertTrue(MainTeleOp.headingReady(Math.toRadians(-179), Math.toRadians(179), 3));
        assertFalse(MainTeleOp.headingReady(Math.toRadians(179), Math.toRadians(-179), 1));
        assertFalse(MainTeleOp.headingReady(Math.toRadians(-179), Math.toRadians(179), 1));
        assertTrue(MainTeleOp.headingReady(Math.PI, -Math.PI, 0.001));
        assertTrue(MainTeleOp.headingReady(-Math.PI, Math.PI, 0.001));
        assertTrue(MainTeleOp.headingReady(4 * Math.PI + 0.1, 0.1, 0.001));
        assertTrue(MainTeleOp.headingReady(0.1, -4 * Math.PI + 0.1, 0.001));
    }

    @Test
    public void headingToleranceIsInclusiveAndMeasuredInDegrees() {
        assertTrue(MainTeleOp.headingReady(0, 0, 0));
        assertTrue(MainTeleOp.headingReady(1.25, 1.25, 0));
        assertFalse(MainTeleOp.headingReady(0, 0.001, 0));
        for (int sign : new int[] {-1, 1}) {
            assertTrue(MainTeleOp.headingReady(sign * Math.PI / 2, 0, 90));
            assertTrue(MainTeleOp.headingReady(0, sign * Math.PI / 2, 90));
            assertFalse(MainTeleOp.headingReady(sign * (Math.PI / 2 + 1e-8), 0, 90));
            assertTrue(MainTeleOp.headingReady(sign * (Math.PI / 2 - 1e-8), 0, 90));
            assertTrue(MainTeleOp.headingReady(sign * Math.PI, 0, 180));
        }
        assertTrue(MainTeleOp.headingReady(Math.toRadians(4), 0, 5));
        assertFalse(MainTeleOp.headingReady(Math.toRadians(6), 0, 5));
    }

    @Test
    public void nonfiniteAnglesAreNeverHeadingReady() {
        double[] invalidAngles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double invalid : invalidAngles) {
            for (double tolerance : new double[] {0, 5, 180}) {
                assertFalse(MainTeleOp.headingReady(invalid, 0, tolerance));
                assertFalse(MainTeleOp.headingReady(0, invalid, tolerance));
                for (double other : invalidAngles) {
                    assertFalse(MainTeleOp.headingReady(invalid, other, tolerance));
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
                    () -> MainTeleOp.headingReady(0, 0, tolerance));
            assertThrows(IllegalArgumentException.class,
                    () -> MainTeleOp.headingReady(Double.NaN, Double.NaN, tolerance));
        }
    }
}
