package com.rmsftc.library.interpolation;

import com.rmsftc.library.math.Polynomial;

public class LagrangeInterp {
    double[] xs, ys;
    Polynomial poly;
    public LagrangeInterp(double[] X, double[] Y) {
        xs = X;
        ys = Y;
    }
    public static Polynomial getPolynomial(double[] X, double[] Y) {
        return new LagrangeInterp(X, Y).buildPoly().poly;
    }
    public LagrangeInterp buildPoly() {
        int degree = xs.length;
        double[][] deltas = new double[degree][degree];
        double [] result = new double[degree];

        for (int i = 0; i < degree; i ++) {
            deltas[i] = getDeltaPolynomial(i).coefs;
        }

        for (int i = 0; i < degree; i ++) {
            //System.out.println(Arrays.toString(deltas[i]));
            result = new Polynomial(result).add(new Polynomial(deltas[i]).scale(ys[i])).coefs;
        }
        poly = new Polynomial(result);
        return this;
    }

    public double getY(double x) {
        return poly.getY(x);
    }

    private Polynomial getDeltaPolynomial (int xpos) {
        Polynomial poly = new Polynomial(new double[]{1});
        double denom = 1;
        for (int i = 0; i < xs.length; i ++) {
            if (i != xpos) {
                Polynomial currentTerm = new Polynomial(new double[]{1, -xs[i]});
                denom *= xs[xpos] - xs[i];
                poly = poly.multiply(currentTerm);
            }
        }
        return poly.scale(1/denom);
    }

}

