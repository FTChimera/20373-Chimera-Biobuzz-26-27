package org.firstinspires.ftc.teamcode.util.autotune;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Ziegler-Nichols candidate generation from relay-test ultimate gain
 * and period, following Aarav Dhawan's FtcAutoTune calculator with the
 * classification corrected: only the P/PI/PID-classic and Pessen
 * entries are textbook Ziegler-Nichols (1942); the "no overshoot" /
 * "some overshoot" PID entries are common-variant tables, and all six
 * PD entries are Aarav variants by analogy (no textbook ZN PD rule
 * exists — ZN 1942 defines P, PI and PID only).
 *
 * <p>All gains assume the parallel form
 * {@code u = kP*e + kI*integral(e) + kD*d(e)/dt} with error in
 * ticks/sec, time in seconds, and output in power units. They do not
 * transfer numerically to a controller with a different form, units,
 * or derivative definition without conversion. Pure Java, no FTC
 * dependency. {@code kF} is deliberately absent: PIDSVA feedforward
 * comes from the SVA fit, not from a single through-origin slope.
 */
public final class ZieglerNichols {

    private ZieglerNichols() {
    }

    /** One candidate gain set. Index 2 is the conservative starting candidate in both families. */
    public static final class Candidate {
        public final int index;
        public final String label;
        public final String variant;
        public final double kP;
        public final double kI;
        public final double kD;

        Candidate(int index, String label, String variant, double kP, double kI, double kD) {
            this.index = index;
            this.label = label;
            this.variant = variant;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        @Override
        public String toString() {
            return label + ": kP=" + kP + ", kI=" + kI + ", kD=" + kD;
        }
    }

    /**
     * Computes the six candidates for the given ultimate gain/period.
     *
     * @param includeIntegral false for the PD-only family (default for
     *                        flywheels: kI = 0); true for the full-PID family.
     */
    public static List<Candidate> computeCandidates(double ku, double tu, boolean includeIntegral) {
        if (!Double.isFinite(ku) || ku <= 0.0) {
            throw new IllegalArgumentException("Ku must be finite and > 0.");
        }
        if (!Double.isFinite(tu) || tu <= 0.0) {
            throw new IllegalArgumentException("Tu must be finite and > 0.");
        }
        List<Candidate> out = new ArrayList<>(6);
        if (!includeIntegral) {
            out.add(new Candidate(0, "P only", "textbook P", 0.50 * ku, 0.0, 0.0));
            out.add(new Candidate(1, "PD (very gentle)", "Aarav variant", 0.30 * ku, 0.0, 0.075 * ku * tu));
            out.add(new Candidate(2, "PD (no overshoot)", "Aarav variant", 0.40 * ku, 0.0, (0.40 * ku) * (tu / 3.0)));
            out.add(new Candidate(3, "PD (some overshoot)", "Aarav variant", 0.60 * ku, 0.0, (0.60 * ku) * (tu / 4.0)));
            out.add(new Candidate(4, "PD (classic ZN) [Aarav label]", "Aarav variant", 0.80 * ku, 0.0, (0.80 * ku) * (tu / 8.0)));
            out.add(new Candidate(5, "PD (aggressive)", "Aarav variant", 1.20 * ku, 0.0, (1.20 * ku) * (tu / 8.0)));
        } else {
            out.add(new Candidate(0, "P only", "textbook P", 0.50 * ku, 0.0, 0.0));
            // PI textbook: Kp=0.45 Ku, Ti=Tu/1.2, Ki=Kp/Ti=0.54 Ku/Tu.
            out.add(new Candidate(1, "PI (textbook)", "textbook ZN", 0.45 * ku, 0.54 * ku / tu, 0.0));
            out.add(new Candidate(2, "PID (no overshoot)", "common variant", 0.20 * ku, 0.40 * ku / tu, (0.20 * ku) * (tu / 3.0)));
            out.add(new Candidate(3, "PID (some overshoot)", "common variant", 0.33 * ku, 0.66 * ku / tu, (0.33 * ku) * (tu / 3.0)));
            out.add(new Candidate(4, "PID (classic ZN)", "textbook ZN", 0.60 * ku, 1.20 * ku / tu, (0.60 * ku) * (tu / 8.0)));
            out.add(new Candidate(5, "PID (Pessen, aggressive)", "textbook Pessen", 0.70 * ku, 1.75 * ku / tu, (0.70 * ku) * (0.15 * tu)));
        }
        return Collections.unmodifiableList(out);
    }
}
