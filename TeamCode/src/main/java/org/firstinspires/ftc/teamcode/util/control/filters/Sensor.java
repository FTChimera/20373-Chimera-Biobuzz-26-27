package org.firstinspires.ftc.teamcode.util.control.filters;

public interface Sensor {
    double[] h(double[] x);                            // nonlinear measurement function
    double[][] jacobianH(double[] x);                  // Jacobian of h wrt x (m x n)
    double[][] R();                                    // measurement noise covariance (m x m)
    double[] residual(double[] z, double[] zPred);     // optional: handle angle wrap, etc.
}
