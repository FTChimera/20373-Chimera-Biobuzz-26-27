package org.firstinspires.ftc.teamcode.library.control;

import java.util.function.Function;

public class PIDFCoefficients<T> {
    private static final int MAX_INFERRED_STATE_LENGTH = 32;

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
        int length = validateStateArray(p, "p");
        validateStateArray(i, "i", length);
        validateStateArray(d, "d", length);
        validateStateArray(f, "f", length);
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        return this;
    }

    public PIDFCoefficients<T> setPIDF(T p, T i, T d, T f) {
        double[] pValues = stateSupplier.apply(p);
        int length = validateStateArray(pValues, "p");
        double[] iValues = stateSupplier.apply(i);
        validateStateArray(iValues, "i", length);
        double[] dValues = stateSupplier.apply(d);
        validateStateArray(dValues, "d", length);
        double[] fValues = stateSupplier.apply(f);
        validateStateArray(fValues, "f", length);

        this.p = pValues;
        this.i = iValues;
        this.d = dValues;
        this.f = fValues;
        return this;
    }

    public PIDFCoefficients<T> setPIDF(double p, double i, double d, double f) {
        int length = resolveStateLength();
        this.p = fillArray(length, p);
        this.i = fillArray(length, i);
        this.d = fillArray(length, d);
        this.f = fillArray(length, f);
        return this;
    }
    public PIDFCoefficients<T> setTolerance(double[] tolerance) {
        validateStateArray(tolerance, "tolerance");
        this.tolerance = tolerance;
        return this;
    }

    public PIDFCoefficients<T> setTolerance(T tolerance) {
        double[] toleranceValues = stateSupplier.apply(tolerance);
        validateStateArray(toleranceValues, "tolerance");
        this.tolerance = toleranceValues;
        return this;
    }

    public PIDFCoefficients<T> setTolerance(double tolerance) {
        int length = resolveStateLength();
        this.tolerance = fillArray(length, tolerance);
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(double[] integralLimit) {
        validateStateArray(integralLimit, "integralLimit");
        this.integralLimit = integralLimit;
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(T integralLimit) {
        double[] integralLimitValues = stateSupplier.apply(integralLimit);
        validateStateArray(integralLimitValues, "integralLimit");
        this.integralLimit = integralLimitValues;
        return this;
    }

    public PIDFCoefficients<T> setIntegralLimit(double integralLimit) {
        int length = resolveStateLength();
        this.integralLimit = fillArray(length, integralLimit);
        return this;
    }

    public void update(double[] error) {
        // This method is blank for now
        // But can be used to implement features like dynamic PIDF tuning based on error magnitude or
        // switching between PIDF coefficients.
    }

    private int validateStateArray(double[] values, String name) {
        int fallbackLength = values == null ? -1 : values.length;
        return validateStateArray(values, name, getExpectedStateLength(fallbackLength));
    }

    private int validateStateArray(double[] values, String name, int expectedLength) {
        if (values == null || values.length == 0) {
            throw new IllegalArgumentException(name + " must be a non-empty array.");
        }
        if (values.length != expectedLength) {
            throw new IllegalArgumentException(name + " must have length " + expectedLength + " but had " + values.length + '.');
        }
        return values.length;
    }

    private int getExpectedStateLength(int fallbackLength) {
        int configuredLength = getConfiguredStateLength();
        if (configuredLength > 0) {
            return configuredLength;
        }

        Integer inferredLength = inferStateLength();
        return inferredLength != null ? inferredLength : fallbackLength;
    }

    private int getConfiguredStateLength() {
        if (p != null) return p.length;
        if (i != null) return i.length;
        if (d != null) return d.length;
        if (f != null) return f.length;
        if (tolerance != null) return tolerance.length;
        if (integralLimit != null) return integralLimit.length;
        return -1;
    }

    private int resolveStateLength() {
        int configuredLength = getConfiguredStateLength();
        if (configuredLength > 0) {
            return configuredLength;
        }

        Integer inferredLength = inferStateLength();
        if (inferredLength != null) {
            return inferredLength;
        }

        throw new IllegalStateException("Unable to determine the PIDF state dimension. Configure an array-based value first or provide suppliers that can infer the state length.");
    }

    private Integer inferStateLength() {
        if (stateSupplier == null || inverseStateSupplier == null) {
            return null;
        }

        for (int candidate = 1; candidate <= MAX_INFERRED_STATE_LENGTH; candidate++) {
            try {
                double[] state = stateSupplier.apply(inverseStateSupplier.apply(new double[candidate]));
                if (state != null && state.length == candidate) {
                    return candidate;
                }
            } catch (RuntimeException ignored) {
                // Try the next candidate length.
            }
        }

        return null;
    }

    private double[] fillArray(int length, double value) {
        double[] values = new double[length];
        for (int idx = 0; idx < length; idx++) {
            values[idx] = value;
        }
        return values;
    }

}
