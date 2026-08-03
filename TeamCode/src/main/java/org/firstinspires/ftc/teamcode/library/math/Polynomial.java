package org.firstinspires.ftc.teamcode.library.math;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Objects;

/**
 * A polynomial whose coefficients are stored highest degree first.
 */
public class Polynomial implements Equation {
    public final double[] coefs;

    public Polynomial(@NonNull double[] coefficients) {
        Objects.requireNonNull(coefficients, "coefficients");
        if (coefficients.length == 0) {
            throw new IllegalArgumentException("A polynomial requires at least one coefficient");
        }
        for (double coefficient : coefficients) {
            if (!Double.isFinite(coefficient)) {
                throw new IllegalArgumentException("Polynomial coefficients must be finite");
            }
        }
        coefs = trimLeadingZeroes(coefficients);
    }

    public int getDegree() {
        return coefs.length - 1;
    }

    @Override
    public double getY(double x) {
        double result = 0;
        for (double coefficient : coefs) {
            result = result * x + coefficient;
        }
        return result;
    }

    @NonNull
    @Override
    public Term[] getTerms() {
        Term[] terms = new Term[coefs.length];
        int degree = getDegree();
        for (int i = 0; i < coefs.length; i++) {
            terms[i] = Term.power(coefs[i], degree - i);
        }
        return terms;
    }

    @NonNull
    @Override
    public Polynomial scale(double k) {
        if (!Double.isFinite(k)) {
            throw new IllegalArgumentException("Scale must be finite");
        }
        double[] scaled = new double[coefs.length];
        for (int i = 0; i < coefs.length; i++) {
            scaled[i] = coefs[i] * k;
        }
        return new Polynomial(scaled);
    }

    @NonNull
    public Polynomial copy() {
        return new Polynomial(coefs);
    }

    @NonNull
    public Polynomial multiply(@NonNull Polynomial other) {
        Objects.requireNonNull(other, "other");
        double[] result = new double[getDegree() + other.getDegree() + 1];
        for (int i = 0; i < coefs.length; i++) {
            for (int j = 0; j < other.coefs.length; j++) {
                result[i + j] += coefs[i] * other.coefs[j];
            }
        }
        return new Polynomial(result);
    }

    @NonNull
    @Override
    public Equation multiply(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        return other instanceof Polynomial
                ? multiply((Polynomial) other)
                : Equation.of(getTerms()).multiply(other);
    }

    @NonNull
    public Polynomial add(@NonNull Polynomial other) {
        Objects.requireNonNull(other, "other");
        int resultLength = Math.max(coefs.length, other.coefs.length);
        double[] result = new double[resultLength];
        addAligned(result, coefs);
        addAligned(result, other.coefs);
        return new Polynomial(result);
    }

    @NonNull
    @Override
    public Equation add(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        return other instanceof Polynomial
                ? add((Polynomial) other)
                : Equation.of(getTerms()).add(other);
    }

    @NonNull
    @Override
    public Polynomial derivative() {
        if (coefs.length == 1) {
            return new Polynomial(new double[]{0});
        }
        double[] derivative = new double[coefs.length - 1];
        int degree = getDegree();
        for (int i = 0; i < derivative.length; i++) {
            derivative[i] = coefs[i] * (degree - i);
        }
        return new Polynomial(derivative);
    }

    @NonNull
    @Override
    public Polynomial indefiniteIntegral() {
        double[] integral = new double[coefs.length + 1];
        int degree = getDegree();
        for (int i = 0; i < coefs.length; i++) {
            integral[i] = coefs[i] / (degree - i + 1);
        }
        return new Polynomial(integral);
    }

    @NonNull
    @Override
    public String toString() {
        String expression = Equation.of(getTerms()).toString();
        return expression.startsWith("f(x) = ") ? expression : "f(x) = " + expression;
    }

    private static void addAligned(double[] result, double[] source) {
        int offset = result.length - source.length;
        for (int i = 0; i < source.length; i++) {
            result[offset + i] += source[i];
        }
    }

    private static double[] trimLeadingZeroes(double[] coefficients) {
        int firstNonZero = 0;
        while (firstNonZero < coefficients.length - 1 && coefficients[firstNonZero] == 0) {
            firstNonZero++;
        }
        return Arrays.copyOfRange(coefficients, firstNonZero, coefficients.length);
    }
}
