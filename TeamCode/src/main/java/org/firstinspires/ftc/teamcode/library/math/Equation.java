package org.firstinspires.ftc.teamcode.library.math;

import androidx.annotation.NonNull;

public interface Equation {
    @NonNull
    String toString();

    Equation derivative();
    Equation indefiniteIntegral();

    Equation add(Equation other);
    Equation multiply(Equation other);
    Equation scale(double k);
    double getY(double x);
    Term[] getTerms();
}
