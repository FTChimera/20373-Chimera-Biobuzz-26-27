package org.firstinspires.ftc.teamcode.util.control.filters;

import static org.junit.Assert.assertArrayEquals;

import com.pedropathing.math.Matrix;

import org.junit.Test;

public class ExtendedKalmanFilterTest {
    @Test
    public void updateUsesMeasurementAndProtectsStateFromExternalMutation() {
        ExtendedKalmanFilter filter = new ExtendedKalmanFilter(
                new double[]{0.0},
                new double[][]{{1.0}},
                new IdentityDynamics(),
                new ScalarSensor(0.01)
        );

        filter.update(new double[]{1.0});

        double[] state = filter.state();
        assertArrayEquals(new double[]{1.0 / 1.01}, state, 1e-9);
        state[0] = 123.0;
        assertArrayEquals(new double[]{1.0 / 1.01}, filter.state(), 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void predictRejectsNegativeTimeStep() {
        ExtendedKalmanFilter filter = new ExtendedKalmanFilter(
                new double[]{0.0},
                new double[][]{{1.0}},
                new IdentityDynamics(),
                new ScalarSensor(1.0)
        );

        filter.predict(-0.01);
    }

    private static final class IdentityDynamics implements Dynamics {
        @Override
        public double[] f(double[] state, double dt) {
            return state;
        }

        @Override
        public Matrix jacobianF(double[] state, double dt) {
            return new Matrix(new double[][]{{1.0}});
        }

        @Override
        public Matrix Q() {
            return new Matrix(new double[][]{{0.0}});
        }
    }

    private static final class ScalarSensor implements Sensor {
        private final double variance;

        private ScalarSensor(double variance) {
            this.variance = variance;
        }

        @Override
        public double[] h(double[] state) {
            return state;
        }

        @Override
        public double[][] jacobianH(double[] state) {
            return new double[][]{{1.0}};
        }

        @Override
        public double[][] R() {
            return new double[][]{{variance}};
        }

        @Override
        public double[] residual(double[] measurement, double[] predictedMeasurement) {
            return new double[]{measurement[0] - predictedMeasurement[0]};
        }
    }
}
