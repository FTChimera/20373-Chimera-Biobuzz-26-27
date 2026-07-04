package org.firstinspires.ftc.teamcode.library.math;

import androidx.annotation.NonNull;

public class Term implements Equation {

    public Term(Class<? extends Equation> type, double coefficient) {} // To be completed later

    @NonNull
    @Override
    public String toString() {
        return "";
    }

    @Override
    public Equation derivative() {
        return null;
    }

    @Override
    public Equation indefiniteIntegral() {
        return null;
    }

    @Override
    public Equation add(Equation other) {
        return null;
    }

    @Override
    public Equation multiply(Equation other) {
        return null;
    }

    @Override
    public Equation scale(double k) {
        return null;
    }

    @Override
    public double getY(double x) {
        return 0;
    }

    @Override
    public Term[] getTerms() {
        return new Term[]{this};
    }
}
