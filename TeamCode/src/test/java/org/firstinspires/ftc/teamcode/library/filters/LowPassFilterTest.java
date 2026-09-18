package org.firstinspires.ftc.teamcode.library.filters;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class LowPassFilterTest {
    @Test
    public void rejectsOutOfRangeAlphaValues() {
        assertThrows(IllegalArgumentException.class, () -> new LowPassFilter(-0.01));
        assertThrows(IllegalArgumentException.class, () -> new LowPassFilter(1.01));
    }

    @Test
    public void updatesStateAndResetsCleanly() {
        LowPassFilter filter = new LowPassFilter(0.25);

        filter.updateState(8.0);
        assertEquals(8.0, filter.getState(), MathUtils.EPSILON);

        filter.updateState(4.0);
        assertEquals(7.0, filter.getState(), MathUtils.EPSILON);

        filter.resetState();
        assertEquals(0.0, filter.getState(), MathUtils.EPSILON);

        filter.updateState(2.0);
        assertEquals(2.0, filter.getState(), MathUtils.EPSILON);
    }

    @Test
    public void kalmanFilterRejectsInvalidNoiseValues() {
        assertThrows(IllegalArgumentException.class, () -> new KalmanFilter(-0.01, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new KalmanFilter(0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new KalmanFilter(0.0, 1.0, 0.0));
    }

    @Test
    public void kalmanFilterConvergesAndResetsCleanly() {
        KalmanFilter filter = new KalmanFilter(0.0, 1.0, 1.0);

        filter.updateState(0.0);
        assertTrue(filter.hasEstimate());
        assertEquals(0.0, filter.getState(), MathUtils.EPSILON);
        assertEquals(1.0, filter.getKalmanGain(), MathUtils.EPSILON);

        filter.updateState(10.0);
        assertEquals(5.0, filter.getState(), MathUtils.EPSILON);
        assertEquals(0.5, filter.getKalmanGain(), MathUtils.EPSILON);
        assertEquals(0.5, filter.getEstimateError(), MathUtils.EPSILON);

        filter.updateState(10.0);
        assertEquals(20.0 / 3.0, filter.getState(), MathUtils.EPSILON);
        assertEquals(1.0 / 3.0, filter.getKalmanGain(), MathUtils.EPSILON);

        filter.resetState();
        assertFalse(filter.hasEstimate());
        assertEquals(0.0, filter.getState(), MathUtils.EPSILON);
        assertEquals(1.0, filter.getEstimateError(), MathUtils.EPSILON);
    }
}
