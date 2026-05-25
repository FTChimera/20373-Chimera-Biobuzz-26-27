package org.firstinspires.ftc.teamcode.library.control;

import java.util.function.Function;

public class PIDFCoefficients<T> {
    public double[] p;
    public double[] i;
    public double[] d;
    public double[] f;
    public double[] tolerance;
    public double[] integralLimit;
    public Function<T, double[]> stateSupplier;
    public Function<double[], T> inverseStateSupplier;
    public PIDFCoefficients(Function<T, double[]> stateSupplier, Function<double[], T> inverseStateSupplier) {
        this.stateSupplier = stateSupplier;
        this.inverseStateSupplier = inverseStateSupplier;
    }

    public PIDFCoefficients<T> setPIDF(double[] p, double[] i, double[] d, double[] f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        return this;
    }

    public PIDFCoefficients<T> setPIDF(T p, T i, T d, T f) {
        this.p = stateSupplier.apply(p);
        this.i = stateSupplier.apply(i);
        this.d = stateSupplier.apply(d);
        this.f = stateSupplier.apply(f);
        return this;
    }

    public PIDFCoefficients<T> setPIDF(double p, double i, double d, double f) {
        int length = stateSupplier.apply(inverseStateSupplier.apply(new double[0])).length;
        this.p = new double[length];
        this.i = new double[length];
        this.d = new double[length];
        this.f = new double[length];
        for (int idx = 0; idx < length; idx++) {
            this.p[idx] = p;
            this.i[idx] = i;
            this.d[idx] = d;
            this.f[idx] = f;
        }
        return this;
    }
    public PIDFCoefficients<T> setTolerance(double[] tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDFCoefficients<T> setTolerance(T tolerance) {
        this.tolerance = stateSupplier.apply(tolerance);
        return this;
    }

    public PIDFCoefficients<T> setTolerance(double tolerance) {
        int length = stateSupplier.apply(inverseStateSupplier.apply(new double[0])).length;
        this.tolerance = new double[length];
        for (int idx = 0; idx < length; idx++) {
            this.tolerance[idx] = tolerance;
        }
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(double[] integralLimit) {
        this.integralLimit = integralLimit;
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(T integralLimit) {
        this.integralLimit = stateSupplier.apply(integralLimit);
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(double integralLimit) {
        int length = stateSupplier.apply(inverseStateSupplier.apply(new double[0])).length;
        this.integralLimit = new double[length];
        for (int idx = 0; idx < length; idx++) {
            this.integralLimit[idx] = integralLimit;
        }
        return this;
    }

    public void update(double[] error) {
        // This method is blank for now
        // But can be used to implement features like dynamic PIDF tuning based on error magnitude or
        // switching between PIDF coefficients.
    }

}
