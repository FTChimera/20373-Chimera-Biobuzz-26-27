package org.firstinspires.ftc.teamcode.library.filters;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

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
}
