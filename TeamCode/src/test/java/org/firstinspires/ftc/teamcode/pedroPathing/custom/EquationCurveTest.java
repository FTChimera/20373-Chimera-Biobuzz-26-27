package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.math.Polynomial;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class EquationCurveTest {
    @Test
    public void mapsTheEquationAndItsDerivativesAcrossPathTime() {
        EquationCurve curve = new EquationCurve(
                new Pose(0, 0),
                new Pose(2, 4),
                new Polynomial(new double[]{1, 0, 0}),
                HeadingInterpolator.constant(Math.PI / 2));

        Pose midpoint = curve.getPose(0.5);
        Vector derivative = curve.getDerivative(0.5);
        Vector secondDerivative = curve.getSecondDerivative(0.5);

        assertEquals(1, midpoint.getX(), MathUtils.EPSILON);
        assertEquals(1, midpoint.getY(), MathUtils.EPSILON);
        assertEquals(Math.PI / 2, midpoint.getHeading(), MathUtils.EPSILON);
        assertEquals(2, derivative.getXComponent(), MathUtils.EPSILON);
        assertEquals(4, derivative.getYComponent(), MathUtils.EPSILON);
        assertEquals(0, secondDerivative.getXComponent(), MathUtils.EPSILON);
        assertEquals(8, secondDerivative.getYComponent(), MathUtils.EPSILON);
    }

    @Test
    public void rejectsVerticalOrMismatchedEquationEndpoints() {
        assertThrows(IllegalArgumentException.class, () -> new EquationCurve(
                new Pose(1, 1),
                new Pose(1, 1),
                new Polynomial(new double[]{1, 0}),
                HeadingInterpolator.constant(0)));

        assertThrows(IllegalArgumentException.class, () -> new EquationCurve(
                new Pose(0, 0),
                new Pose(1, 2),
                new Polynomial(new double[]{1, 0}),
                HeadingInterpolator.constant(0)));
    }
}
