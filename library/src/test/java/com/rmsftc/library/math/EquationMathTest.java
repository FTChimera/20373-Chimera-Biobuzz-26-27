package com.rmsftc.library.math;

import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class EquationMathTest {
    @Test
    public void polynomialAddsDifferentDegreesAndReturnsEveryTerm() {
        Polynomial left = new Polynomial(new double[]{2, -3, 1});
        Polynomial right = new Polynomial(new double[]{4, -2});

        Polynomial sum = left.add(right);

        assertArrayEquals(new double[]{2, 1, -1}, sum.coefs, MathUtils.EPSILON);
        assertEquals(3, left.getTerms().length);
        assertEquals(1, left.getTerms()[2].getY(10), MathUtils.EPSILON);
    }

    @Test
    public void trigonometricTermsEvaluateAndDifferentiate() {
        Equation equation = Term.sin(2).add(Term.cos(-3));

        assertEquals(-3, equation.getY(0), MathUtils.EPSILON);
        assertEquals(2, equation.derivative().getY(0), MathUtils.EPSILON);
        assertEquals(equation.getY(0.4), equation.indefiniteIntegral().derivative().getY(0.4), MathUtils.EPSILON);
    }

    @Test
    public void productRuleWorksForPolynomialAndTrigFactors() {
        Equation equation = Term.power(3, 2).multiply(Term.sin(2));
        double x = 0.5;
        double expectedDerivative = 12 * x * Math.sin(x) + 6 * x * x * Math.cos(x);

        assertEquals(6 * x * x * Math.sin(x), equation.getY(x), MathUtils.EPSILON);
        assertEquals(expectedDerivative, equation.derivative().getY(x), MathUtils.EPSILON);
    }

    @Test
    public void mixedPolynomialAndTrigAlgebraRemainsEvaluatable() {
        Polynomial polynomial = new Polynomial(new double[]{1, 0, -1});
        Equation mixed = polynomial.add((Equation) Term.sin(1));

        assertEquals(3 + Math.sin(2), mixed.getY(2), MathUtils.EPSILON);
        assertEquals(4 + Math.cos(2), mixed.derivative().getY(2), MathUtils.EPSILON);
    }

    @Test
    public void unsupportedProductsFailExplicitlyInsteadOfReturningNull() {
        Equation product = Term.sin(1).multiply(Term.cos(1));

        assertThrows(UnsupportedOperationException.class, product::indefiniteIntegral);
    }
}
