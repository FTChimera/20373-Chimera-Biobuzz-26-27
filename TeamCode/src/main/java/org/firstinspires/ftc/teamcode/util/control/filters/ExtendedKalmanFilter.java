package org.firstinspires.ftc.teamcode.util.control.filters;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.Arrays;
import java.util.Objects;

public class ExtendedKalmanFilter {

    public double[] x;
    public Matrix P;

    public final Dynamics dynamics;
    public final Sensor sensor;

    public ExtendedKalmanFilter(double[] x0, double[][] P0, Dynamics dynamics, Sensor sensor) {
        Objects.requireNonNull(x0, "x0");
        Objects.requireNonNull(P0, "P0");
        if (x0.length == 0) {
            throw new IllegalArgumentException("State must have at least one element.");
        }

        this.x = Arrays.copyOf(x0, x0.length);
        this.P = new Matrix(P0);
        if (P.numRows() != x0.length || P.numCols() != x0.length) {
            throw new IllegalArgumentException("Initial covariance must be square with one row/column per state.");
        }

        this.dynamics = Objects.requireNonNull(dynamics, "dynamics");
        this.sensor = Objects.requireNonNull(sensor, "sensor");
    }

    // Predict step: propagate state and covariance
    public void predict(double dt) {
        double[] priorState = Arrays.copyOf(x, x.length);

        x = Objects.requireNonNull(dynamics.f(priorState, dt), "Dynamics returned a null state.");
        requireVectorLength("Predicted state", x, priorState.length);

        Matrix F = requireSquareMatrix("State Jacobian", dynamics.jacobianF(priorState, dt), priorState.length);
        Matrix Q = requireSquareMatrix("Process noise covariance", dynamics.Q(), priorState.length);
        P = symmetrize(F.times(P).times(F.transpose()).plus(Q));
    }

    // Update step: incorporate measurement and reduce uncertainty
    public void update(double[] z) {
        Objects.requireNonNull(z, "z");
        double[] zPred = Objects.requireNonNull(sensor.h(x), "Sensor returned a null predicted measurement.");
        requireVectorLength("Predicted measurement", zPred, z.length);

        double[] innovation = Objects.requireNonNull(sensor.residual(z, zPred), "Sensor returned a null residual.");
        requireVectorLength("Measurement residual", innovation, z.length);

        Matrix H = requireMeasurementJacobian(sensor.jacobianH(x), z.length, x.length);
        Matrix R = requireSquareMatrix("Measurement noise covariance", new Matrix(sensor.R()), z.length);

        Vector y = asVectorObject(innovation);
        Matrix Ht = H.transpose();
        Matrix S = H.times(P).times(Ht).plus(R);
        Matrix K = P.times(Ht).times(S.invert());

        x = vectorToArray(asVectorObject(x).plus(K.times(y)));

        Matrix I = Matrix.identity(x.length);
        Matrix left = I.minus(K.times(H));
        P = symmetrize(left.times(P).times(left.transpose()).plus(K.times(R).times(K.transpose())));
    }

    public void step(double[] z, double dt) {
        predict(dt);
        update(z);
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

    private static void requireVectorLength(String name, double[] vector, int expectedLength) {
        if (vector.length != expectedLength) {
            throw new IllegalArgumentException(name + " must have length " + expectedLength + ".");
        }
    }
}
