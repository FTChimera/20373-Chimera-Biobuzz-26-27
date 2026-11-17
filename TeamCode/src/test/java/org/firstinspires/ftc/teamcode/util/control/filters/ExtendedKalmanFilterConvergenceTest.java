package org.firstinspires.ftc.teamcode.util.control.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Matrix;

import org.junit.Test;

/**
 * Pure-JVM tests for {@link ExtendedKalmanFilter} — the predict/update
 * core behind {@code EKFPinpoint} (the robot's EKF localizer). No
 * hardware: a 2-state [position, velocity] constant-velocity model
 * with a position-only sensor, so the filter must converge toward the
 * measurement and its covariance must shrink.
 *
 * <p>The portfolio's claim that the EKF "smooths the robot's position"
 * rests on this predict/update cycle: predict grows the covariance
 * (uncertainty accumulates while moving), update pulls the state
 * toward the measurement and shrinks the covariance (the measurement
 * resolves uncertainty). These tests pin that cycle down.
 */
public class ExtendedKalmanFilterConvergenceTest {

    private static final double TOLERANCE = 1e-3;

    /** Constant-velocity dynamics: x = [position, velocity]. */
    private static final class ConstantVelocity implements Dynamics {
        @Override
        public double[] f(double[] x, double dt) {
            return new double[]{x[0] + x[1] * dt, x[1]};
        }

        @Override
        public Matrix jacobianF(double[] x, double dt) {
            return new Matrix(new double[][]{{1.0, dt}, {0.0, 1.0}});
        }

        @Override
        public Matrix Q() {
            return new Matrix(new double[][]{{0.01, 0.0}, {0.0, 0.01}});
        }
    }

    /** Position-only sensor: measures x[0] with variance 0.1. */
    private static final class PositionSensor implements Sensor {
        @Override
        public double[] h(double[] x) {
            return new double[]{x[0]};
        }

        @Override
        public double[][] jacobianH(double[] x) {
            return new double[][]{{1.0, 0.0}};
        }

        @Override
        public double[][] R() {
            return new double[][]{{0.1}};
        }

        @Override
        public double[] residual(double[] z, double[] zPred) {
            return new double[]{z[0] - zPred[0]};
        }
    }

    private static ExtendedKalmanFilter filter(double[] x0, double[][] p0) {
        return new ExtendedKalmanFilter(x0, p0, new ConstantVelocity(), new PositionSensor());
    }

    private static double trace(Matrix m) {
        double sum = 0.0;
        for (int i = 0; i < Math.min(m.numRows(), m.numCols()); i++) {
            sum += m.get(i, i);
        }
        return sum;
    }

    @Test
    public void predictMovesPositionByVelocity() {
        ExtendedKalmanFilter ekf = filter(
                new double[]{0.0, 2.0},
                new double[][]{{1.0, 0.0}, {0.0, 1.0}});

        ekf.predict(0.5);

        double[] state = ekf.state();
        assertEquals(1.0, state[0], TOLERANCE);
        assertEquals(2.0, state[1], TOLERANCE);
        // Predict grows the position variance (uncertainty accumulates).
        assertTrue("P[0][0] should grow on predict, was " + ekf.covariance().get(0, 0),
                ekf.covariance().get(0, 0) > 1.0);
    }

    @Test
    public void updateMovesStateTowardMeasurementAndShrinksCovariance() {
        ExtendedKalmanFilter ekf = filter(
                new double[]{0.0, 0.0},
                new double[][]{{1.0, 0.0}, {0.0, 1.0}});
        double traceBefore = trace(ekf.covariance());

        ekf.update(new double[]{10.0});

        double[] state = ekf.state();
        // Kalman gain K = P/(P+R) = 1/1.1, so the state moves 10/1.1 toward 10.
        assertEquals(10.0 / 1.1, state[0], TOLERANCE);
        assertEquals(0.0, state[1], TOLERANCE);
        // Covariance shrinks: P[0][0] goes 1.0 -> ~0.091, trace 2.0 -> ~1.09.
        assertEquals(1.0 / 11.0, ekf.covariance().get(0, 0), TOLERANCE);
        assertTrue("Covariance trace should shrink on update",
                trace(ekf.covariance()) < traceBefore);
    }

    @Test
    public void repeatedUpdatesConvergeTowardMeasurement() {
        ExtendedKalmanFilter ekf = filter(
                new double[]{0.0, 0.0},
                new double[][]{{1.0, 0.0}, {0.0, 1.0}});

        double previousPosition = 0.0;
        double previousVariance = ekf.covariance().get(0, 0);
        for (int i = 0; i < 5; i++) {
            ekf.update(new double[]{10.0});
            double position = ekf.state()[0];
            double variance = ekf.covariance().get(0, 0);
            assertTrue("Each update should move the state closer to 10",
                    position > previousPosition);
            assertTrue("Each update should shrink the position variance",
                    variance < previousVariance);
            previousPosition = position;
            previousVariance = variance;
        }
        assertTrue("After 5 updates the state should be close to 10, was " + ekf.state()[0],
                ekf.state()[0] > 9.5);
        assertTrue("After 5 updates the variance should be small, was " + ekf.covariance().get(0, 0),
                ekf.covariance().get(0, 0) < 0.05);
    }
}
