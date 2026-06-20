package org.firstinspires.ftc.teamcode.library.math;

import androidx.annotation.NonNull;

import java.util.Locale;

/**
 Length 1 polynomial – 0th degree, length 2 polynomial - 1st degree, etc.
 0th term – highest degree term

 */
public class Polynomial {
    public double[] coefs;
    public int getDegree() {
        return coefs.length - 1;
    }
    public double getY(double x) {
        int degree = coefs.length - 1;
        double result = 0;
        for (int i = 0; i <= degree; i ++) {
            double xPow = Math.pow(x, degree - i);
            double term = xPow * coefs[i];
            result += term;
        }
        return result;
    }

    public Polynomial(double[] a) {
        coefs = a.clone();
    }

    public Polynomial scale(double k) {
        double [] b = new double[coefs.length];
        for (int i = 0; i <= getDegree(); i ++) {
            b[i] = coefs[i]*k;
        }
        return new Polynomial(b);
    }

    public Polynomial copy() {return new Polynomial(coefs.clone());}

    public Polynomial multiply(Polynomial other) {
        int degA = getDegree();
        int degB = other.getDegree();
        double[] result = new double[degA + degB + 1];
        for (int i = 0; i <= degA; i ++) {
            for (int j = 0; j <= degB; j ++) {
                result[i + j] += coefs[i]*other.coefs[j];
            }
        }
        return new Polynomial(result);
    }

    public Polynomial add(Polynomial other) {
        int degree = getDegree();
        double[] c = new double[degree+1];
        for (int i = 0; i <= degree; i ++) {
            c[i] = coefs[i] + other.coefs[i];
        }
        return new Polynomial(c);
    }
    @NonNull
    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        double[] coefficients = coefs.clone();
        int degree = coefficients.length;
        for (int i = 0; i < degree; i ++) {
            if (coefficients[i] > 0 && !s.toString().isEmpty()) {
                s.append("  +  ");
            }
            if (coefficients[i] < 0) {
                coefficients[i]*= -1;
                s.append("  -  ");
            }
            if (coefficients[i] != 0) {
                if (coefficients[i] == (int) coefficients[i]) {
                    if (coefficients[i] != 1 || (coefficients[i] == 1 && degree-i-1 == 0)) {
                        s.append((int) coefficients[i]);
                    }
                }
                else {
                    s.append(String.format(Locale.US, "%.2f", coefficients[i]));
                }
            }
            if (coefficients[i] != 0) {
                if (degree-i-1 == 1) {
                    s.append("x");
                }
                else if (degree-i-1 > 1) {
                    s.append("x^").append(degree - i - 1);
                }
            }

        }
        return "f(x) = " + s;
    }

    // IMPLEMENT METHODS FOR DERIVATIVE AND INTEGRAL
     public Polynomial derivative() {
         double[] dCoefs = new double[coefs.length - 1];
         int degree = getDegree();
         for (int i = 0; i < degree; i++) {
             dCoefs[i] = coefs[i] * (degree - i);
         }
         return new Polynomial(dCoefs);
     }
    public Polynomial indefiniteIntegral() {
        double[] iCoefs = new double[coefs.length + 1];
        int degree = getDegree();
        for (int i = 0; i <= degree; i++) {
            iCoefs[i] = coefs[i] / (degree - i + 1);
        }
        return new Polynomial(iCoefs);
    }
}