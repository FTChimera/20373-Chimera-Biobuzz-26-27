package org.firstinspires.ftc.teamcode.util.control.filters;

import com.pedropathing.math.Matrix;

public interface Dynamics {
    double[] f(double[] x, double dt);

    Matrix jacobianF(double[] x, double dt);

    /**
     * Returns the discrete process-noise covariance for the given time step.
     * Implementations that use fixed per-step noise can implement only {@link #Q()}.
     */
    default Matrix Q(double dt) {
        return Q();
    }

    /**
     * Returns a fixed discrete process-noise covariance.
     */
    Matrix Q();

    /**
     * Converts a state back to its canonical representation after prediction or correction.
     * This is useful for circular state elements such as headings.
     */
    default double[] normalizeState(double[] state) {
        return state;
    }
}
