package com.rmsftc.library.interpolation;

import com.rmsftc.library.math.MathUtils;
import com.rmsftc.library.math.Polynomial;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

public class InterpolationTest {
    @Test
    public void linearInterpolationUsesBoundsAndInteriorSegments() {
        LinearInterp interpolation = new LinearInterp()
                .min(100.0)
                .point(0.2, 950.0)
                .point(0.5, 1100.0)
                .point(0.8, 1400.0)
                .max(2000.0);

        assertEquals(100.0, interpolation.calculate(0.1), MathUtils.EPSILON);
        assertEquals(1025.0, interpolation.calculate(0.35), MathUtils.EPSILON);
        assertEquals(2000.0, interpolation.calculate(0.9), MathUtils.EPSILON);
        assertEquals(15.0, LinearInterp.lerp(10.0, 20.0, 0.5), MathUtils.EPSILON);
    }

    @Test
    public void lagrangeInterpolationBuildsExpectedQuadratic() {
        double[] xs = {0.0, 1.0, 2.0};
        double[] ys = {1.0, 3.0, 7.0};

        Polynomial polynomial = LagrangeInterp.getPolynomial(xs, ys);
        LagrangeInterp interpolation = new LagrangeInterp(xs, ys).buildPoly();

        assertArrayEquals(new double[]{1.0, 1.0, 1.0}, polynomial.coefs, MathUtils.EPSILON);
        assertEquals(4.75, interpolation.getY(1.5), MathUtils.EPSILON);
    }
}
