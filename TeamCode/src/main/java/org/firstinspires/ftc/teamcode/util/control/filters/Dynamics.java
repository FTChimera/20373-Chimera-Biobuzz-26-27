package org.firstinspires.ftc.teamcode.util.control.filters;

import com.pedropathing.math.Matrix;

public interface Dynamics {
    double[] f(double[] x, double dt);                 // nonlinear state transition
    Matrix jacobianF(double[] x, double dt);      // Jacobian of f wrt x (n x n)
    Matrix Q();                                    // process noise covariance (n x n)
}
