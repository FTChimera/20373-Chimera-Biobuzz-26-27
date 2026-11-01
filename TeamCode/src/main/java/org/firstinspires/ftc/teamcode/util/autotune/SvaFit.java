package org.firstinspires.ftc.teamcode.util.autotune;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Three-parameter least-squares fit for the flywheel plant
 * {@code y = kS * sgn(v) + kV * v + kA * a}.
 *
 * <p>Pure Java, no FTC dependency. When voltage compensation is on,
 * {@code y} is volts ({@code power * batteryVoltage} per sample, using
 * each sample's concurrent voltage reading); when it is off, {@code y}
 * is power and the result is valid only near the recorded battery
 * voltage. The fit uses column-scaled normal equations with Gaussian
 * elimination and partial pivoting: the velocity column is scaled by
 * {@code max|v|} and the acceleration column by {@code max|a|} before
 * solving, then unscaled. Unscaled normal equations are ill-scaled
 * (raw condition number ~3621 on the design data vs ~5.6 scaled) —
 * scaling is mandatory, not cosmetic.
 *
 * <p>Quasistatic-only or single-rate data are rank-deficient and are
 * rejected, never silently reduced to a two-parameter fit labelled
 * SVA. Negative {@code kS}/{@code kA} beyond small tolerances are
 * handled as specified in the fit result: a small negative kS is
 * refitted with kS fixed at zero and flagged; a materially negative
 * kA rejects the dynamic phase.
 */
public final class SvaFit {

    private SvaFit() {
    }

    /** One characterisation sample. Acceleration is the profile acceleration for ramp samples, or the filtered-derivative estimate for step samples — never raw differentiated velocity. */
    public static final class Sample {
        public final double velocityTicksPerSec;
        public final double accelerationTicksPerSecSq;
        public final double power;
        public final double batteryVoltage;
        public final String phase;

        public Sample(double velocityTicksPerSec, double accelerationTicksPerSecSq,
                      double power, double batteryVoltage, String phase) {
            this.velocityTicksPerSec = velocityTicksPerSec;
            this.accelerationTicksPerSecSq = accelerationTicksPerSecSq;
            this.power = power;
            this.batteryVoltage = batteryVoltage;
            this.phase = phase;
        }
    }

    /** Fit outcome. {@code accepted} is false when the constants must not be used. */
    public static final class FitResult {
        public final boolean accepted;
        public final String rejectReason;
        public final double kS;
        public final double kV;
        public final double kA;
        public final boolean kSClampedZero;
        public final boolean kARejected;
        public final double rmse;
        public final Map<String, Double> rmseByPhase;
        public final double rSquared;
        public final double scaledConditionNumber;
        public final int samplesUsed;
        public final int samplesExcluded;
        public final List<String> warnings;
        public final boolean fittedInVolts;

        FitResult(boolean accepted, String rejectReason,
                  double kS, double kV, double kA,
                  boolean kSClampedZero, boolean kARejected,
                  double rmse, Map<String, Double> rmseByPhase, double rSquared,
                  double scaledConditionNumber, int samplesUsed, int samplesExcluded,
                  List<String> warnings, boolean fittedInVolts) {
            this.accepted = accepted;
            this.rejectReason = rejectReason;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kSClampedZero = kSClampedZero;
            this.kARejected = kARejected;
            this.rmse = rmse;
            this.rmseByPhase = rmseByPhase;
            this.rSquared = rSquared;
            this.scaledConditionNumber = scaledConditionNumber;
            this.samplesUsed = samplesUsed;
            this.samplesExcluded = samplesExcluded;
            this.warnings = warnings;
            this.fittedInVolts = fittedInVolts;
        }
    }

    /**
     * Fits kS/kV/kA from the given samples.
     *
     * <p>Samples with {@code |v| < 50} ticks/sec are excluded
     * (stiction-dominated sgn region). Samples with non-finite
     * velocity/acceleration/power are excluded. When
     * {@code voltageCompensationOn} is true, samples with non-finite or
     * non-positive battery voltage are excluded (the caller must apply
     * its voltage fallback ladder before supplying samples).
     */
    public static FitResult fit(List<Sample> samples, boolean voltageCompensationOn) {
        List<String> warnings = new ArrayList<>();
        if (samples == null || samples.isEmpty()) {
            return rejected("No samples supplied.", warnings, voltageCompensationOn, 0, 0);
        }
        List<Sample> used = new ArrayList<>();
        int excluded = 0;
        for (Sample s : samples) {
            if (s == null || !Double.isFinite(s.velocityTicksPerSec)
                    || !Double.isFinite(s.accelerationTicksPerSecSq) || !Double.isFinite(s.power)) {
                excluded++;
                continue;
            }
            if (Math.abs(s.velocityTicksPerSec) < 50.0) {
                excluded++;
                continue;
            }
            if (voltageCompensationOn && (!Double.isFinite(s.batteryVoltage) || s.batteryVoltage <= 0.0)) {
                excluded++;
                continue;
            }
            used.add(s);
        }
        if (used.size() < 5) {
            return rejected("Only " + used.size() + " usable samples after exclusions; need at least 5.",
                    warnings, voltageCompensationOn, used.size(), excluded);
        }
        // Distinct acceleration magnitudes: need at least 2 distinct nonzero |a| values.
        List<Double> accelMagnitudes = new ArrayList<>();
        for (Sample s : used) {
            double mag = Math.abs(s.accelerationTicksPerSecSq);
            if (mag > 1e-9) {
                boolean seen = false;
                for (double m : accelMagnitudes) {
                    if (Math.abs(m - mag) <= 1e-6) {
                        seen = true;
                        break;
                    }
                }
                if (!seen) {
                    accelMagnitudes.add(mag);
                }
            }
        }
        boolean hasQuasistatic = false;
        for (Sample s : used) {
            if (Math.abs(s.accelerationTicksPerSecSq) <= 1e-9) {
                hasQuasistatic = true;
                break;
            }
        }
        if (!hasQuasistatic || accelMagnitudes.size() < 2) {
            return rejected("Insufficient data: need quasistatic (a=0) samples and at least 2 distinct nonzero |a| values; "
                    + "got quasistatic=" + hasQuasistatic + ", distinct |a| count=" + accelMagnitudes.size()
                    + ". Single-rate or quasistatic-only data are rank-deficient.",
                    warnings, voltageCompensationOn, used.size(), excluded);
        }

        double[] y = new double[used.size()];
        for (int i = 0; i < used.size(); i++) {
            Sample s = used.get(i);
            y[i] = voltageCompensationOn ? s.power * s.batteryVoltage : s.power;
        }
        double[][] x = new double[used.size()][3];
        for (int i = 0; i < used.size(); i++) {
            Sample s = used.get(i);
            x[i][0] = s.velocityTicksPerSec > 0.0 ? 1.0 : -1.0;
            x[i][1] = s.velocityTicksPerSec;
            x[i][2] = s.accelerationTicksPerSecSq;
        }

        double[] beta = solveScaledLeastSquares(x, y);
        if (beta == null) {
            return rejected("Scaled normal equations are singular or near-singular; characterisation data do not identify all three parameters.",
                    warnings, voltageCompensationOn, used.size(), excluded);
        }
        double kS = beta[0];
        double kV = beta[1];
        double kA = beta[2];
        double condition = scaledConditionNumber(x);

        // Sanity checks, in fitted units (volts when compensation is on, power when off).
        double kSNegTol = voltageCompensationOn ? 0.24 : 0.02;
        double kANegTol = voltageCompensationOn ? 0.00024 : 0.00002;
        boolean kSClampedZero = false;
        if (kS < -kSNegTol) {
            return rejected("Fitted kS=" + kS + " is materially negative; fit rejected.",
                    warnings, voltageCompensationOn, used.size(), excluded);
        }
        if (kS < 0.0) {
            // Refit with kS fixed at zero (2-parameter fit for kV/kA).
            double[] beta2 = solveScaledLeastSquaresTwoParam(x, y);
            if (beta2 == null) {
                return rejected("kS was slightly negative and the kS=0 refit is singular; fit rejected.",
                        warnings, voltageCompensationOn, used.size(), excluded);
            }
            kS = 0.0;
            kV = beta2[0];
            kA = beta2[1];
            kSClampedZero = true;
            warnings.add("kS was slightly negative and was clamped to 0 with a 2-parameter refit.");
        }
        if (kA < -kANegTol) {
            FitResult r = buildResult(false, "Fitted kA=" + kA + " is materially negative; dynamic phase rejected, kA must not be used.",
                    kS, kV, kA, kSClampedZero, true, used, y, x, condition, warnings, voltageCompensationOn, excluded);
            return r;
        }
        if (kA < 0.0) {
            kA = 0.0;
            warnings.add("kA was slightly negative and was clamped to 0; re-run the dynamic phase before calling the constants complete.");
        }
        if (kV <= 0.0) {
            return rejected("Fitted kV=" + kV + " is not positive; fit rejected.",
                    warnings, voltageCompensationOn, used.size(), excluded);
        }
        if (condition > 1e6) {
            // Condition of scaled X^T X; sqrt is the condition of scaled X (~1e3 threshold).
            warnings.add("Scaled normal-equation condition number " + condition + " is high; fit may be ill-conditioned. Check that both quasistatic and dynamic phases ran.");
        }
        return buildResult(true, "", kS, kV, kA, kSClampedZero, false,
                used, y, x, condition, warnings, voltageCompensationOn, excluded);
    }

    private static FitResult rejected(String reason, List<String> warnings, boolean fittedInVolts, int used, int excluded) {
        return new FitResult(false, reason, Double.NaN, Double.NaN, Double.NaN,
                false, false, Double.NaN, Collections.emptyMap(), Double.NaN,
                Double.NaN, used, excluded, warnings, fittedInVolts);
    }

    private static FitResult buildResult(boolean accepted, String rejectReason,
                                         double kS, double kV, double kA,
                                         boolean kSClampedZero, boolean kARejected,
                                         List<Sample> used, double[] y, double[][] x,
                                         double condition, List<String> warnings,
                                         boolean fittedInVolts, int excluded) {
        // Residuals with the final (possibly clamped) coefficients, in fitted units.
        double sumSq = 0.0;
        double sumY = 0.0;
        for (double v : y) {
            sumY += v;
        }
        double meanY = y.length == 0 ? 0.0 : sumY / y.length;
        double totalSq = 0.0;
        Map<String, List<Double>> residualsByPhase = new LinkedHashMap<>();
        for (int i = 0; i < used.size(); i++) {
            double predicted = kS * x[i][0] + kV * x[i][1] + kA * x[i][2];
            double residual = y[i] - predicted;
            sumSq += residual * residual;
            totalSq += (y[i] - meanY) * (y[i] - meanY);
            String phase = used.get(i).phase == null ? "unknown" : used.get(i).phase;
            residualsByPhase.computeIfAbsent(phase, k -> new ArrayList<>()).add(residual);
        }
        double rmse = used.isEmpty() ? Double.NaN : Math.sqrt(sumSq / used.size());
        double rSquared = totalSq <= 0.0 ? Double.NaN : 1.0 - sumSq / totalSq;
        Map<String, Double> rmseByPhase = new LinkedHashMap<>();
        for (Map.Entry<String, List<Double>> e : residualsByPhase.entrySet()) {
            double s = 0.0;
            for (double r : e.getValue()) {
                s += r * r;
            }
            rmseByPhase.put(e.getKey(), Math.sqrt(s / e.getValue().size()));
        }
        return new FitResult(accepted, rejectReason, kS, kV, kA,
                kSClampedZero, kARejected, rmse, rmseByPhase, rSquared,
                condition, used.size(), excluded, warnings, fittedInVolts);
    }

    /** Solves the scaled 3-parameter least squares problem; returns [kS, kV, kA] or null if singular. */
    private static double[] solveScaledLeastSquares(double[][] x, double[] y) {
        double vRef = 0.0;
        double aRef = 0.0;
        for (double[] row : x) {
            vRef = Math.max(vRef, Math.abs(row[1]));
            aRef = Math.max(aRef, Math.abs(row[2]));
        }
        if (vRef <= 0.0 || aRef <= 0.0) {
            return null;
        }
        double[][] scaled = new double[x.length][3];
        for (int i = 0; i < x.length; i++) {
            scaled[i][0] = x[i][0];
            scaled[i][1] = x[i][1] / vRef;
            scaled[i][2] = x[i][2] / aRef;
        }
        double[][] ata = new double[3][3];
        double[] aty = new double[3];
        for (int i = 0; i < scaled.length; i++) {
            for (int r = 0; r < 3; r++) {
                aty[r] += scaled[i][r] * y[i];
                for (int c = 0; c < 3; c++) {
                    ata[r][c] += scaled[i][r] * scaled[i][c];
                }
            }
        }
        double[] betaScaled = solveLinearSystem(ata, aty);
        if (betaScaled == null) {
            return null;
        }
        return new double[]{betaScaled[0], betaScaled[1] / vRef, betaScaled[2] / aRef};
    }

    /** Solves the scaled 2-parameter (kV, kA) problem with kS fixed at zero; returns [kV, kA] or null if singular. */
    private static double[] solveScaledLeastSquaresTwoParam(double[][] x, double[] y) {
        double vRef = 0.0;
        double aRef = 0.0;
        for (double[] row : x) {
            vRef = Math.max(vRef, Math.abs(row[1]));
            aRef = Math.max(aRef, Math.abs(row[2]));
        }
        if (vRef <= 0.0 || aRef <= 0.0) {
            return null;
        }
        double[][] ata = new double[2][2];
        double[] aty = new double[2];
        for (int i = 0; i < x.length; i++) {
            double[] row = new double[]{x[i][1] / vRef, x[i][2] / aRef};
            for (int r = 0; r < 2; r++) {
                aty[r] += row[r] * y[i];
                for (int c = 0; c < 2; c++) {
                    ata[r][c] += row[r] * row[c];
                }
            }
        }
        double[] betaScaled = solveLinearSystem(ata, aty);
        if (betaScaled == null) {
            return null;
        }
        return new double[]{betaScaled[0] / vRef, betaScaled[1] / aRef};
    }

    /** Gaussian elimination with partial pivoting; returns null when a pivot is below 1e-12. */
    static double[] solveLinearSystem(double[][] a, double[] b) {
        int n = b.length;
        double[][] m = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, m[i], 0, n);
            m[i][n] = b[i];
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                    pivot = row;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                return null;
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            for (int row = col + 1; row < n; row++) {
                double factor = m[row][col] / m[col][col];
                for (int c = col; c <= n; c++) {
                    m[row][c] -= factor * m[col][c];
                }
            }
        }
        double[] out = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = m[i][n];
            for (int c = i + 1; c < n; c++) {
                sum -= m[i][c] * out[c];
            }
            out[i] = sum / m[i][i];
        }
        return out;
    }

    /** Frobenius-norm condition estimate of the scaled X^T X matrix (sqrt gives the scaled-X condition). */
    private static double scaledConditionNumber(double[][] x) {
        double vRef = 0.0;
        double aRef = 0.0;
        for (double[] row : x) {
            vRef = Math.max(vRef, Math.abs(row[1]));
            aRef = Math.max(aRef, Math.abs(row[2]));
        }
        if (vRef <= 0.0 || aRef <= 0.0) {
            return Double.NaN;
        }
        double[][] ata = new double[3][3];
        for (double[] row : x) {
            double[] s = new double[]{row[0], row[1] / vRef, row[2] / aRef};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    ata[r][c] += s[r] * s[c];
                }
            }
        }
        double[][] inv = invert3(ata);
        if (inv == null) {
            return Double.POSITIVE_INFINITY;
        }
        return frobeniusNorm(ata) * frobeniusNorm(inv);
    }

    private static double frobeniusNorm(double[][] m) {
        double sum = 0.0;
        for (double[] row : m) {
            for (double v : row) {
                sum += v * v;
            }
        }
        return Math.sqrt(sum);
    }

    private static double[][] invert3(double[][] a) {
        double[][] m = new double[3][6];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(a[i], 0, m[i], 0, 3);
            m[i][3 + i] = 1.0;
        }
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                    pivot = row;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                return null;
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            double div = m[col][col];
            for (int c = 0; c < 6; c++) {
                m[col][c] /= div;
            }
            for (int row = 0; row < 3; row++) {
                if (row == col) {
                    continue;
                }
                double factor = m[row][col];
                for (int c = 0; c < 6; c++) {
                    m[row][c] -= factor * m[col][c];
                }
            }
        }
        double[][] inv = new double[3][3];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(m[i], 3, inv[i], 0, 3);
        }
        return inv;
    }
}
