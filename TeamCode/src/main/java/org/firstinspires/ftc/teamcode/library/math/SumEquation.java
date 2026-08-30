package org.firstinspires.ftc.teamcode.library.math;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Internal normalized representation for a sum of symbolic terms.
 */
final class SumEquation implements Equation {
    private final Term[] terms;

    private SumEquation(Term[] terms) {
        this.terms = terms;
    }

    @NonNull
    static Equation of(@NonNull Term... input) {
        Objects.requireNonNull(input, "terms");
        List<Term> normalized = new ArrayList<>();
        for (Term term : input) {
            Objects.requireNonNull(term, "term");
            if (term.isZero()) {
                continue;
            }

            boolean combined = false;
            for (int i = 0; i < normalized.size(); i++) {
                Term existing = normalized.get(i);
                if (existing.hasSameFactors(term)) {
                    Term replacement = existing.withCoefficient(
                            existing.getCoefficient() + term.getCoefficient());
                    if (replacement.isZero()) {
                        normalized.remove(i);
                    } else {
                        normalized.set(i, replacement);
                    }
                    combined = true;
                    break;
                }
            }
            if (!combined) {
                normalized.add(term);
            }
        }

        if (normalized.isEmpty()) {
            return Term.constant(0);
        }
        if (normalized.size() == 1) {
            return normalized.get(0);
        }
        return new SumEquation(normalized.toArray(new Term[0]));
    }

    @NonNull
    @Override
    public Equation derivative() {
        return map(Operation.DERIVATIVE);
    }

    @NonNull
    @Override
    public Equation indefiniteIntegral() {
        return map(Operation.INTEGRAL);
    }

    @NonNull
    @Override
    public Equation add(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        Term[] otherTerms = other.getTerms();
        Term[] result = new Term[terms.length + otherTerms.length];
        System.arraycopy(terms, 0, result, 0, terms.length);
        System.arraycopy(otherTerms, 0, result, terms.length, otherTerms.length);
        return of(result);
    }

    @NonNull
    @Override
    public Equation multiply(@NonNull Equation other) {
        Objects.requireNonNull(other, "other");
        List<Term> products = new ArrayList<>();
        for (Term term : terms) {
            Term[] termProducts = term.multiply(other).getTerms();
            for (Term product : termProducts) {
                products.add(product);
            }
        }
        return of(products.toArray(new Term[0]));
    }

    @NonNull
    @Override
    public Equation scale(double k) {
        if (!Double.isFinite(k)) {
            throw new IllegalArgumentException("Scale must be finite");
        }
        Term[] scaled = new Term[terms.length];
        for (int i = 0; i < terms.length; i++) {
            scaled[i] = terms[i].scale(k);
        }
        return of(scaled);
    }

    @Override
    public double getY(double x) {
        double result = 0;
        for (Term term : terms) {
            result += term.getY(x);
        }
        return result;
    }

    @NonNull
    @Override
    public Term[] getTerms() {
        return terms.clone();
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("f(x) = ");
        for (int i = 0; i < terms.length; i++) {
            String term = terms[i].toString();
            if (i > 0 && !term.startsWith("-")) {
                result.append(" + ");
            } else if (i > 0) {
                result.append(" - ");
                term = term.substring(1);
            }
            result.append(term);
        }
        return result.toString();
    }

    private Equation map(Operation operation) {
        List<Term> mapped = new ArrayList<>();
        for (Term term : terms) {
            Equation result = operation == Operation.DERIVATIVE
                    ? term.derivative()
                    : term.indefiniteIntegral();
            Term[] resultTerms = result.getTerms();
            mapped.addAll(Arrays.asList(resultTerms));
        }
        return of(mapped.toArray(new Term[0]));
    }

    private enum Operation {
        DERIVATIVE,
        INTEGRAL
    }
}
