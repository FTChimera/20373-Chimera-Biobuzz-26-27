package org.firstinspires.ftc.teamcode.util.interpolation;

import org.firstinspires.ftc.teamcode.util.math.Polynomial;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map;


public class LagrangeInterp implements Interpolator {
    private Polynomial built;
    private final ArrayList<Map.Entry<Double, Double>> map = new ArrayList<>();

    public LagrangeInterp(Collection<Map.Entry<Double, Double>> map) {
        editAndBuildPoly(map);
    }

    public void editAndBuildPoly(Collection<Map.Entry<Double, Double>> newMap) {
        this.map.clear();
        this.map.addAll(newMap);
        this.map.sort(Comparator.comparingDouble(Map.Entry::getKey));
        buildPolynomial();
    }

    private void buildPolynomial() {
        int n = map.size(); // number of interpolation points
        double[][] deltas = new double[n][n];
        double [] result = new double[n];

        for (int i = 0; i < n; i ++) {
            deltas[i] = getDeltaPolynomial(i).coefs;
        }

        for (int i = 0; i < n; i ++) {
            //System.out.println(Arrays.toString(deltas[i]));
            result = new Polynomial(result).add(new Polynomial(deltas[i]).scale(map.get(i).getValue())).coefs;
        }
        built = new Polynomial(result);
    }

    private Polynomial getDeltaPolynomial (int xpos) {
        Polynomial poly = new Polynomial(new double[]{1});
        double denom = 1;
        for (int i = 0; i < map.size(); i ++) {
            if (i != xpos) {
                Polynomial currentTerm = new Polynomial(new double[]{1, -map.get(i).getKey()});
                denom *= map.get(xpos).getKey() - map.get(i).getKey();
                poly = poly.multiply(currentTerm);
            }
        }
        return poly.scale(1/denom);
    }

    public Polynomial getBuiltPolynomial() {
        return built;
    }

    @Override
    public double interpolate(double input) {
        return built.getY(input);
    }
}
