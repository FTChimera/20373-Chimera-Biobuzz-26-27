package org.firstinspires.ftc.teamcode.library.math;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MathUtilsTest {
    @Test
    public void headingBetweenCoordinatesMatchesExpectedAngles() {
        assertEquals(Math.PI / 4.0, MathUtils.headingBetween(0.0, 0.0, 2.0, 2.0), MathUtils.EPSILON);
        assertEquals(-Math.PI / 2.0, MathUtils.headingBetween(3.0, 4.0, 3.0, -1.0), MathUtils.EPSILON);
    }

    @Test
    public void headingDeltaWrapsAcrossThePiBoundary() {
        assertEquals(Math.toRadians(20.0),
                MathUtils.headingDelta(Math.toRadians(170.0), Math.toRadians(-170.0)),
                MathUtils.EPSILON);
        assertEquals(-Math.toRadians(20.0),
                MathUtils.headingDelta(Math.toRadians(-170.0), Math.toRadians(170.0)),
                MathUtils.EPSILON);
    }
}
