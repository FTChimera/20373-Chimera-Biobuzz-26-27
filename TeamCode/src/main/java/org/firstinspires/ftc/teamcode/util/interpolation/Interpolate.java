package org.firstinspires.ftc.teamcode.util.interpolation;

import org.firstinspires.ftc.teamcode.util.math.Equation;

public final class Interpolate {
    public static double linear(double t, double a, double b) {
        return func(t, a, b, Equation.IDENTITY);
    }

    public static double func(double t, double a, double b, Equation equation) {
        return a + equation.getY(t) * (b-a);
    }
}
