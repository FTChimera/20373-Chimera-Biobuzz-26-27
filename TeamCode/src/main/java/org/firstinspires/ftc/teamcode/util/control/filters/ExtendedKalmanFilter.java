package org.firstinspires.ftc.teamcode.util.control.filters;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.Arrays;
import java.util.Objects;

public class ExtendedKalmanFilter {

    private double[] x;
    private Matrix P;

    private final Dynamics dynamics;
    private final Sensor sensor;

    public ExtendedKalmanFilter(double[] x0, double[][] P0, Dynamics dynamics, Sensor sensor) {
        Objects.requireNonNull(x0, "x0");
        Objects.requireNonNull(P0, "P0");
        if (x0.length == 0) {
            throw new IllegalArgumentException("State must have at least one element.");
        }

        this.dynamics = Objects.requireNonNull(dynamics, "dynamics");
        this.sensor = Objects.requireNonNull(sensor, "sensor");

        this.x = normalizeState(Arrays.copyOf(x0, x0.length), x0.length);
        this.P = symmetrize(new Matrix(P0));
        if (P.numRows() != x0.length || P.numCols() != x0.length) {
            throw new IllegalArgumentException("Initial covariance must be square with one row/column per state.");
        }
        requireFiniteVector("Initial state", x);
        requireFiniteMatrix("Initial covariance", P);
    }

    public void predict(double dt) {
        requireNonnegativeFinite("dt", dt);
        double[] priorState = Arrays.copyOf(x, x.length);

        double[] predictedState = Objects.requireNonNull(dynamics.f(priorState, dt), "Dynamics returned a null state.");
        requireVectorLength("Predicted state", predictedState, priorState.length);
        predictedState = normalizeState(predictedState, priorState.length);
        requireFiniteVector("Predicted state", predictedState);
        Matrix F = requireSquareMatrix("State Jacobian", dynamics.jacobianF(priorState, dt), priorState.length);
        Matrix Q = requireSquareMatrix("Process noise covariance", dynamics.Q(dt), priorState.length);
        requireFiniteMatrix("State Jacobian", F);
        requireFiniteMatrix("Process noise covariance", Q);

        Matrix predictedCovariance = symmetrize(F.times(P).times(F.transpose()).plus(Q));
        requireFiniteMatrix("Predicted covariance", predictedCovariance);

        x = predictedState;
        P = predictedCovariance;
    }

    public void update(double[] z) {
        Objects.requireNonNull(z, "z");
        requireFiniteVector("Measurement", z);
        double[] zPred = Objects.requireNonNull(sensor.h(x), "Sensor returned a null predicted measurement.");
        requireVectorLength("Predicted measurement", zPred, z.length);
        requireFiniteVector("Predicted measurement", zPred);

        double[] innovation = Objects.requireNonNull(sensor.residual(z, zPred), "Sensor returned a null residual.");
        requireVectorLength("Measurement residual", innovation, z.length);
        requireFiniteVector("Measurement residual", innovation);

        Matrix H = requireMeasurementJacobian(sensor.jacobianH(x), z.length, x.length);
        Matrix R = requireSquareMatrix("Measurement noise covariance", new Matrix(sensor.R()), z.length);
        requireFiniteMatrix("Measurement Jacobian", H);
        requireFiniteMatrix("Measurement noise covariance", R);

        Vector y = asVectorObject(innovation);
        Matrix Ht = H.transpose();
        Matrix S = H.times(P).times(Ht).plus(R);
        Matrix SInverse;
        try {
            SInverse = S.invert();
        } catch (IllegalArgumentException exception) {
            throw new IllegalStateException("Innovation covariance is singular; check measurement noise covariance.", exception);
        }
        Matrix K = P.times(Ht).times(SInverse);

        double[] correctedState = vectorToArray(asVectorObject(x).plus(K.times(y)));
        correctedState = normalizeState(correctedState, x.length);
        requireFiniteVector("Corrected state", correctedState);

        Matrix I = Matrix.identity(correctedState.length);
        Matrix left = I.minus(K.times(H));
        Matrix correctedCovariance = symmetrize(left.times(P).times(left.transpose()).plus(K.times(R).times(K.transpose())));
        requireFiniteMatrix("Corrected covariance", correctedCovariance);

        x = correctedState;
        P = correctedCovariance;
    }

    public void step(double[] z, double dt) {
        predict(dt);
        update(z);
    }

    public double[] state() {
        return Arrays.copyOf(x, x.length);
    }

    public Matrix covariance() {
        double[][] values = new double[P.numRows()][P.numCols()];
        for (int row = 0; row < P.numRows(); row++) {
            for (int col = 0; col < P.numCols(); col++) {
                values[row][col] = P.get(row, col);
            }
        }
        return new Matrix(values);
    }

    private static Vector asVectorObject(double[] values) {
        return new Vector(values);
    }

    private static double[] vectorToArray(Vector vector) {
        double[] values = new double[vector.size()];
        for (int i = 0; i < values.length; i++) {
            values[i] = vector.get(i);
        }
        return values;
    }

    private static Matrix symmetrize(Matrix matrix) {
        if (matrix.numRows() != matrix.numCols()) {
            throw new IllegalArgumentException("Matrix must be square to symmetrize.");
        }

        double[][] values = new double[matrix.numRows()][matrix.numCols()];
        for (int row = 0; row < matrix.numRows(); row++) {
            for (int col = row; col < matrix.numCols(); col++) {
                double average = 0.5 * (matrix.get(row, col) + matrix.get(col, row));
                values[row][col] = average;
                values[col][row] = average;
            }
        }
        return new Matrix(values);
    }

    private static Matrix requireMeasurementJacobian(double[][] matrix, int measurementDim, int stateDim) {
        Objects.requireNonNull(matrix, "Measurement Jacobian cannot be null.");
        Matrix H = new Matrix(matrix);
        if (H.numRows() != measurementDim || H.numCols() != stateDim) {
            throw new IllegalArgumentException("Measurement Jacobian must be measurementDim x stateDim.");
        }
        return H;
    }

    private static Matrix requireSquareMatrix(String name, Matrix matrix, int dimension) {
        Objects.requireNonNull(matrix, name + " cannot be null.");
        if (matrix.numRows() != dimension || matrix.numCols() != dimension) {
            throw new IllegalArgumentException(name + " must be " + dimension + "x" + dimension + ".");
        }
        return matrix;
    }

    private double[] normalizeState(double[] state, int expectedLength) {
        double[] normalizedState = Objects.requireNonNull(
                dynamics.normalizeState(Arrays.copyOf(state, state.length)),
                "Dynamics returned a null normalized state."
        );
        requireVectorLength("Normalized state", normalizedState, expectedLength);
        return normalizedState;
    }

    private static void requireVectorLength(String name, double[] vector, int expectedLength) {
        if (vector.length != expectedLength) {
            throw new IllegalArgumentException(name + " must have length " + expectedLength + ".");
        }
    }

    private static void requireNonnegativeFinite(String name, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative.");
        }
    }

    private static void requireFiniteVector(String name, double[] vector) {
        for (double value : vector) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(name + " must contain only finite values.");
            }
        }
    }

    private static void requireFiniteMatrix(String name, Matrix matrix) {
        for (int row = 0; row < matrix.numRows(); row++) {
            for (int col = 0; col < matrix.numCols(); col++) {
                if (!Double.isFinite(matrix.get(row, col))) {
                    throw new IllegalArgumentException(name + " must contain only finite values.");
                }
            }
        }
    }
}
