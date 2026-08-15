package com.rmsftc.library.math;

import androidx.annotation.NonNull;

/**
 * A symbolic single-variable equation that can be evaluated at {@code x}.
 *
 * <p>Implementations use a sum of {@link Term terms} as their common algebraic representation.
 * An equation may therefore combine polynomial and trigonometric terms without returning
 * placeholder values.</p>
 */
public interface Equation {
    @NonNull
    Equation derivative();

    /**
     * Returns an antiderivative with a zero integration constant.
     *
     * @throws UnsupportedOperationException when the equation has no supported elementary
     *                                       antiderivative.
     */
    @NonNull
    Equation indefiniteIntegral();

    @NonNull
    Equation add(@NonNull Equation other);

    @NonNull
    Equation multiply(@NonNull Equation other);

    @NonNull
    Equation scale(double k);

    double getY(double x);

    /**
     * Returns the additive terms that make up this equation.
     */
    @NonNull
    Term[] getTerms();

    /**
     * Creates an equation from additive terms, combining like terms and discarding zero terms.
     */
    @NonNull
    static Equation of(@NonNull Term... terms) {
        return SumEquation.of(terms);
    }
}
