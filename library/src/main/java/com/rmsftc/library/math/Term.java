package com.rmsftc.library.math;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

/**
 * An immutable multiplicative term of the form {@code coefficient * f(x)}.
 *
 * <p>Terms can contain several factors, allowing expressions such as
 * {@code 3*x^2*sin(x)}. {@link Function} supplies the built-in trigonometric and
 * elementary-function tokens; powers are created with {@link #power(double, double)}.</p>
 */
public final class Term implements Equation {
    public enum Function {
        POWER("x"),
        SINE("sin(x)"),
        COSINE("cos(x)"),
        TANGENT("tan(x)"),
        SECANT("sec(x)"),
        COSECANT("csc(x)"),
        COTANGENT("cot(x)"),
        EXPONENTIAL("exp(x)"),
        LOGARITHM("ln(x)"),
        LOG_ABS_COSINE("ln|cos(x)|"),
        LOG_ABS_SINE("ln|sin(x)|"),
        LOG_ABS_SECANT_PLUS_TANGENT("ln|sec(x) + tan(x)|"),
        LOG_ABS_COSECANT_MINUS_COTANGENT("ln|csc(x) - cot(x)|");

        private final String symbol;

        Function(String symbol) {
            this.symbol = symbol;
        }

        private double evaluate(double x) {
            switch (this) {
                case SINE:
                    return Math.sin(x);
                case COSINE:
                    return Math.cos(x);
                case TANGENT:
                    return Math.tan(x);
                case SECANT:
                    return 1.0 / Math.cos(x);
                case COSECANT:
                    return 1.0 / Math.sin(x);
                case COTANGENT:
                    return 1.0 / Math.tan(x);
                case EXPONENTIAL:
                    return Math.exp(x);
                case LOGARITHM:
                    return Math.log(x);
                case LOG_ABS_COSINE:
                    return Math.log(Math.abs(Math.cos(x)));
                case LOG_ABS_SINE:
                    return Math.log(Math.abs(Math.sin(x)));
                case LOG_ABS_SECANT_PLUS_TANGENT:
                    return Math.log(Math.abs(1.0 / Math.cos(x) + Math.tan(x)));
                case LOG_ABS_COSECANT_MINUS_COTANGENT:
                    return Math.log(Math.abs(1.0 / Math.sin(x) - 1.0 / Math.tan(x)));
                default:
                    throw new IllegalStateException("Unhandled function token: " + this);
            }
        }
    }

    private static final Comparator<Factor> FACTOR_ORDER = new Comparator<Factor>() {
        @Override
        public int compare(Factor left, Factor right) {
            int typeComparison = left.function.ordinal() - right.function.ordinal();
            return typeComparison != 0
                    ? typeComparison
                    : Double.compare(left.exponent, right.exponent);
        }
    };

    private final double coefficient;
    private final List<Factor> factors;

    /**
     * Creates {@code coefficient * function(x)}.
     */
    public Term(@NonNull Function function, double coefficient) {
        this(coefficient, Collections.singletonList(new Factor(
                requireNonPower(function), 0)));
    }

    private Term(double coefficient, List<Factor> factors) {
        if (!Double.isFinite(coefficient)) {
            throw new IllegalArgumentException("Coefficient must be finite");
        }
        this.coefficient = coefficient;
        this.factors = Collections.unmodifiableList(normalizeFactors(factors));
    }

    /**
     * Creates {@code coefficient * x^exponent}.
     */
    @NonNull
    public static Term power(double coefficient, double exponent) {
        if (!Double.isFinite(exponent)) {
            throw new IllegalArgumentException("Exponent must be finite");
        }
        return new Term(coefficient, Collections.singletonList(powerFactor(exponent)));
    }

    /**
     * Creates a constant term.
     */
    @NonNull
    public static Term constant(double coefficient) {
        return new Term(coefficient, Collections.<Factor>emptyList());
    }

    @NonNull
    public static Term sin(double coefficient) {
        return new Term(Function.SINE, coefficient);
    }

    @NonNull
    public static Term cos(double coefficient) {
        return new Term(Function.COSINE, coefficient);
    }

    @NonNull
    public static Term tan(double coefficient) {
        return new Term(Function.TANGENT, coefficient);
    }

    @NonNull
    public static Term exp(double coefficient) {
        return new Term(Function.EXPONENTIAL, coefficient);
    }

    public double getCoefficient() {
        return coefficient;
    }

    @NonNull
    @Override
    public Equation derivative() {
        if (coefficient == 0 || factors.isEmpty()) {
            return constant(0);
        }

        List<Term> derivatives = new ArrayList<>();
        for (int i = 0; i < factors.size(); i++) {
            List<Factor> remainingFactors = new ArrayList<>(factors);
            Factor differentiatedFactor = remainingFactors.remove(i);
            Equation remainingTerm = new Term(coefficient, remainingFactors);
            addTerms(derivatives, remainingTerm.multiply(differentiatedFactor.derivative()).getTerms());
        }
        return Equation.of(derivatives.toArray(new Term[0]));
    }

    @NonNull
    @Override
    public Equation indefiniteIntegral() {
        if (coefficient == 0) {
            return constant(0);
        }
        if (factors.isEmpty()) {
            return power(coefficient, 1);
        }

        if (factors.size() == 1) {
            Factor factor = factors.get(0);
            if (factor.isPower()) {
                if (factor.exponent == -1) {
                    return new Term(Function.LOGARITHM, coefficient);
                }
                return power(coefficient / (factor.exponent + 1), factor.exponent + 1);
            }

            switch (factor.function) {
                case SINE:
                    return cos(-coefficient);
                case COSINE:
                    return sin(coefficient);
                case TANGENT:
                    return new Term(Function.LOG_ABS_COSINE, -coefficient);
                case SECANT:
                    return new Term(Function.LOG_ABS_SECANT_PLUS_TANGENT, coefficient);
                case COSECANT:
                    return new Term(Function.LOG_ABS_COSECANT_MINUS_COTANGENT, coefficient);
                case COTANGENT:
                    return new Term(Function.LOG_ABS_SINE, coefficient);
                case EXPONENTIAL:
                    return exp(coefficient);
                case LOGARITHM:
                    return new Term(coefficient, Collections.singletonList(powerFactor(1)))
                            .multiply(new Term(Function.LOGARITHM, 1))
                            .add(power(-coefficient, 1));
                default:
                    break;
            }
        }

        if (hasFactors(Function.SECANT, Function.TANGENT)) {
            return new Term(Function.SECANT, coefficient);
        }
        if (hasFactors(Function.COSECANT, Function.COTANGENT)) {
            return new Term(Function.COSECANT, -coefficient);
        }
        if (hasFactors(Function.SECANT, Function.SECANT)) {
            return new Term(Function.TANGENT, coefficient);
        }
        if (hasFactors(Function.COSECANT, Function.COSECANT)) {
            return new Term(Function.COTANGENT, -coefficient);
        }

        throw new UnsupportedOperationException(
                "No elementary antiderivative is defined for term " + this);
    }

    @NonNull
    @Override
    public Equation add(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        List<Term> terms = new ArrayList<>();
        terms.add(this);
        addTerms(terms, other.getTerms());
        return Equation.of(terms.toArray(new Term[0]));
    }

    @NonNull
    @Override
    public Equation multiply(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        List<Term> products = new ArrayList<>();
        for (Term otherTerm : other.getTerms()) {
            products.add(multiplyTerm(otherTerm));
        }
        return Equation.of(products.toArray(new Term[0]));
    }

    @NonNull
    @Override
    public Term scale(double k) {
        if (!Double.isFinite(k)) {
            throw new IllegalArgumentException("Scale must be finite");
        }
        return new Term(coefficient * k, factors);
    }

    @Override
    public double getY(double x) {
        double value = coefficient;
        for (Factor factor : factors) {
            value *= factor.evaluate(x);
        }
        return value;
    }

    @NonNull
    @Override
    public Term[] getTerms() {
        return new Term[]{this};
    }

    @Override
    public String toString() {
        if (factors.isEmpty()) {
            return format(coefficient);
        }

        StringBuilder result = new StringBuilder();
        if (coefficient == -1) {
            result.append("-");
        } else if (coefficient != 1) {
            result.append(format(coefficient)).append("*");
        }

        for (int i = 0; i < factors.size(); i++) {
            if (i > 0) {
                result.append("*");
            }
            result.append(factors.get(i));
        }
        return result.toString();
    }

    boolean hasSameFactors(Term other) {
        return factors.equals(other.factors);
    }

    Term withCoefficient(double newCoefficient) {
        return new Term(newCoefficient, factors);
    }

    boolean isZero() {
        return coefficient == 0;
    }

    private Term multiplyTerm(Term other) {
        List<Factor> combinedFactors = new ArrayList<>(factors);
        combinedFactors.addAll(other.factors);
        return new Term(coefficient * other.coefficient, combinedFactors);
    }

    private boolean hasFactors(Function first, Function second) {
        return factors.size() == 2
                && ((factors.get(0).function == first && factors.get(1).function == second)
                || (factors.get(0).function == second && factors.get(1).function == first));
    }

    private static void addTerms(List<Term> destination, Term[] source) {
        Collections.addAll(destination, source);
    }

    private static List<Factor> normalizeFactors(List<Factor> source) {
        List<Factor> normalized = new ArrayList<>();
        double totalPower = 0;
        for (Factor factor : source) {
            if (factor.isPower()) {
                totalPower += factor.exponent;
            } else {
                normalized.add(factor);
            }
        }
        if (totalPower != 0) {
            normalized.add(powerFactor(totalPower));
        }
        Collections.sort(normalized, FACTOR_ORDER);
        return normalized;
    }

    private static Factor powerFactor(double exponent) {
        return new Factor(Function.POWER, exponent);
    }

    private static Function requireNonPower(Function function) {
        Function nonNullFunction = Objects.requireNonNull(function, "function");
        if (nonNullFunction == Function.POWER) {
            throw new IllegalArgumentException("Use Term.power(coefficient, exponent) for powers");
        }
        return nonNullFunction;
    }

    private static String format(double value) {
        if (value == Math.rint(value)) {
            return String.format(Locale.US, "%.0f", value);
        }
        return String.format(Locale.US, "%.6f", value)
                .replaceAll("0+$", "")
                .replaceAll("\\.$", "");
    }

    private static final class Factor {
        private final Function function;
        private final double exponent;

        private Factor(Function function, double exponent) {
            this.function = function;
            this.exponent = exponent;
        }

        private boolean isPower() {
            return function == Function.POWER;
        }

        private double evaluate(double x) {
            return isPower() ? Math.pow(x, exponent) : function.evaluate(x);
        }

        @NonNull
        private Equation derivative() {
            if (isPower()) {
                return exponent == 0 ? constant(0) : power(exponent, exponent - 1);
            }

            switch (function) {
                case SINE:
                    return cos(1);
                case COSINE:
                    return sin(-1);
                case TANGENT:
                    return new Term(Function.SECANT, 1).multiply(new Term(Function.SECANT, 1));
                case SECANT:
                    return new Term(Function.SECANT, 1).multiply(new Term(Function.TANGENT, 1));
                case COSECANT:
                    return new Term(Function.COSECANT, -1)
                            .multiply(new Term(Function.COTANGENT, 1));
                case COTANGENT:
                    return new Term(Function.COSECANT, -1)
                            .multiply(new Term(Function.COSECANT, 1));
                case EXPONENTIAL:
                    return exp(1);
                case LOGARITHM:
                    return power(1, -1);
                case LOG_ABS_COSINE:
                    return tan(-1);
                case LOG_ABS_SINE:
                    return new Term(Function.COTANGENT, 1);
                case LOG_ABS_SECANT_PLUS_TANGENT:
                    return new Term(Function.SECANT, 1);
                case LOG_ABS_COSECANT_MINUS_COTANGENT:
                    return new Term(Function.COSECANT, 1);
                default:
                    throw new IllegalStateException("Unhandled function token: " + function);
            }
        }

        @Override
        public String toString() {
            if (isPower()) {
                return exponent == 1 ? "x" : "x^" + format(exponent);
            }
            return function.symbol;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof Factor)) {
                return false;
            }
            Factor factor = (Factor) other;
            return function == factor.function
                    && Double.doubleToLongBits(exponent) == Double.doubleToLongBits(factor.exponent);
        }

        @Override
        public int hashCode() {
            return Objects.hash(function, exponent);
        }
    }
}
