package org.firstinspires.ftc.teamcode.library.control;

import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class PIDFControlTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void coefficientArraysRejectMismatchedStateDimensions() {
        org.firstinspires.ftc.teamcode.library.control.PIDFCoefficients<StateVector> coefficients = twoStateCoefficients();

        assertThrows(IllegalArgumentException.class, () -> coefficients.setPIDF(
                new double[]{1.0, 2.0},
                new double[]{0.0},
                new double[]{0.0, 0.0},
                new double[]{0.0, 0.0}
        ));
    }

    @Test
    public void scalarConfigurationReusesStateLengthAndExportsSdkCoefficients() {
        org.firstinspires.ftc.teamcode.library.control.PIDFCoefficients<StateVector> coefficients = twoStateCoefficients();
        coefficients.setPIDF(
                new double[]{1.0, 2.0},
                new double[]{3.0, 4.0},
                new double[]{5.0, 6.0},
                new double[]{7.0, 8.0}
        );
        coefficients.setTolerance(0.1);
        coefficients.setIntegralLimit(2.0);
        coefficients.setPIDF(3.0, 4.0, 5.0, 6.0);

        assertArrayEquals(new double[]{3.0, 3.0}, coefficients.p, EPSILON);
        assertArrayEquals(new double[]{4.0, 4.0}, coefficients.i, EPSILON);
        assertArrayEquals(new double[]{5.0, 5.0}, coefficients.d, EPSILON);
        assertArrayEquals(new double[]{6.0, 6.0}, coefficients.f, EPSILON);
        assertArrayEquals(new double[]{0.1, 0.1}, coefficients.tolerance, EPSILON);
        assertArrayEquals(new double[]{2.0, 2.0}, coefficients.integralLimit, EPSILON);

        DoublePIDFCoeffs doubles = new DoublePIDFCoeffs(1.0, 2.0, 3.0, 4.0);
        com.qualcomm.robotcore.hardware.PIDFCoefficients sdk = doubles.getAsFtcSdk();
        assertEquals(1.0, sdk.p, EPSILON);
        assertEquals(2.0, sdk.i, EPSILON);
        assertEquals(3.0, sdk.d, EPSILON);
        assertEquals(4.0, sdk.f, EPSILON);
    }

    @Test
    public void controllerProducesProportionalOutputAndStopsInsideTolerance() {
        DoublePIDFCoeffs coefficients = new DoublePIDFCoeffs(2.0, 0.0, 0.0, 0.0);
        coefficients.setTolerance(0.05);
        coefficients.setIntegralLimit(1.0);

        DoublePIDFController controller = new DoublePIDFController(coefficients);
        controller.setTarget(1.0);

        controller.update(0.0);
        assertEquals(2.0, controller.getOutput(), EPSILON);
        assertFalse(controller.atTarget());

        controller.update(0.98);
        assertTrue(controller.atTarget());
        assertEquals(0.0, controller.getOutput(), EPSILON);
        assertEquals(0.02, controller.lastError(), 1e-6);

        controller.reset();
        assertFalse(controller.atTarget());
        assertEquals(0.0, controller.dt, EPSILON);
    }

    private org.firstinspires.ftc.teamcode.library.control.PIDFCoefficients<StateVector> twoStateCoefficients() {
        return new org.firstinspires.ftc.teamcode.library.control.PIDFCoefficients<>(
                state -> state.values,
                values -> {
                    if (values.length != 2) {
                        throw new IllegalArgumentException("Expected a two-state vector");
                    }
                    return new StateVector(values);
                }
        );
    }

    private static final class StateVector {
        private final double[] values;

        private StateVector(double[] values) {
            this.values = values;
        }
    }
}
