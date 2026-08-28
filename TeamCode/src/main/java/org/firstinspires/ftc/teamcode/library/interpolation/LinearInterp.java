package org.firstinspires.ftc.teamcode.library.interpolation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * A class that allows you to create a dynamic calculator for any mechanism. You can add points to the calculator, and it will interpolate between them to calculate the output. You can also set a minimum and maximum output for edge cases.

 * Example usage:
 * LinearInterp velocityCalculator = new LinearInterp()
 *     .min(900)
 *     .point(0.22, 950)
 *     .point(0.34, 1050)
 *     .point(0.4, 1100)
 *     .point(0.76, 1260)
 *     .max(1320);

 * double velocity = velocityCalculator.calculate(0.5); // will interpolate between 1100 and 1260
 */



@SuppressWarnings("SpellCheckingInspection")
public class LinearInterp {
    List<Double> keys, values;
    double min=Double.NaN, max=Double.NaN;
    public LinearInterp() {
        keys = new ArrayList<>();
        values = new ArrayList<>();
    }

    public LinearInterp point(double x, double y) {
        keys.add(x);
        values.add(y);
        return this;
    }

    /**
     * Output for lower edge case (x <= lowest key)
     */
    public LinearInterp min(double y) {
        this.min = y;
        return this;
    }
    /**
     * Output for higher edge case (x >= highest key)
     */
    public LinearInterp max(double y) {
        this.max = y;
        return this;
    }
    public double calculate(double x) {
        if (Double.isNaN(min) || Double.isNaN(max) || keys.isEmpty()) {
            throw new IllegalStateException("Points not configured");
        }
        List<Double> sortedVals = new ArrayList<>(keys);
        sortedVals.sort(null);
        if (!Arrays.equals(keys.toArray(new Double[0]), sortedVals.toArray(new Double[0]))) {
            throw new IllegalStateException("Inputs not sorted");
        }
        if (x <= this.keys.get(0)) {
            return this.min; // edge case lower
        } else if (x >= this.keys.get(this.keys.size() - 1)) {
            return this.max; // edge case higher
        }
        for (int i = 0; i < this.keys.size() - 1; i++) {
            double d0 = this.keys.get(i);
            double d1 = this.keys.get(i + 1);
            // linear interpolation
            if (x >= d0 && x <= d1) {
                double r0 = this.values.get(i);
                double r1 = this.values.get(i+1);
                double t = (x - d0) / (d1 - d0);
                return lerp(r0, r1, t);
            }
        }
        return Double.NaN; // Should never reach here
    }

    public static double lerp(double min, double max, double t) {
        return min + t*(max - min);
    }
}
