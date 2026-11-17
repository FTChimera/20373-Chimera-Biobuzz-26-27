package org.firstinspires.ftc.teamcode.util.autotune;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Pure-JVM tests for {@link SvaFit} — the three-parameter least-squares
 * fit (y = kS*sgn(v) + kV*v + kA*a) behind the shooter PIDSVA tuner.
 * No hardware, no FTC SDK: samples are synthetic, generated from known
 * constants, so the fit must recover them.
 *
 * <p>The constants here are the main flywheel's tuned values, in volts
 * (voltage compensation on, battery 12 V), so power =
 * (kS + kV*v + kA*a) / 12. The portfolio's claim that the tuner
 * "accurately reaches target velocity" rests on this fit identifying
 * the plant; these tests pin that identification down.
 */
public class SvaFitTest {

    private static final double TRUE_KS = 0.143507;
    private static final double TRUE_KV = 0.00543311;
    private static final double TRUE_KA = 0.00595835;
    private static final double BATTERY_VOLTAGE = 12.0;
    private static final double TOLERANCE = 1e-6;

    /** Builds one synthetic sample from the true constants (volts, comp on). */
    private static SvaFit.Sample sample(double velocityTicksPerSec,
                                        double accelerationTicksPerSecSq,
                                        String phase) {
        double yVolts = TRUE_KS * Math.signum(velocityTicksPerSec)
                + TRUE_KV * velocityTicksPerSec
                + TRUE_KA * accelerationTicksPerSecSq;
        double power = yVolts / BATTERY_VOLTAGE;
        return new SvaFit.Sample(velocityTicksPerSec, accelerationTicksPerSecSq,
                power, BATTERY_VOLTAGE, phase);
    }

    /** Well-conditioned data: quasistatic (a=0) plus two distinct |a| rates. */
    private static List<SvaFit.Sample> goodSamples() {
        List<SvaFit.Sample> samples = new ArrayList<>();
        // Quasistatic (a = 0): identifies kS + kV.
        samples.add(sample(300, 0, "quasistatic"));
        samples.add(sample(600, 0, "quasistatic"));
        samples.add(sample(900, 0, "quasistatic"));
        samples.add(sample(1200, 0, "quasistatic"));
        samples.add(sample(1500, 0, "quasistatic"));
        // Dynamic: two distinct |a| (500 and 1000) identify kA.
        samples.add(sample(800, 500, "dynamic"));
        samples.add(sample(1000, 1000, "dynamic"));
        samples.add(sample(1200, 500, "dynamic"));
        samples.add(sample(600, -500, "dynamic"));
        samples.add(sample(1400, 500, "dynamic"));
        return samples;
    }

    @Test
    public void recoversKnownConstantsFromSyntheticSamples() {
        SvaFit.FitResult result = SvaFit.fit(goodSamples(), true);

        assertTrue("Fit should be accepted: " + result.rejectReason, result.accepted);
        assertEquals(TRUE_KS, result.kS, TOLERANCE);
        assertEquals(TRUE_KV, result.kV, TOLERANCE);
        assertEquals(TRUE_KA, result.kA, TOLERANCE);
        assertEquals(10, result.samplesUsed);
        assertEquals(0, result.samplesExcluded);
        assertTrue("RMSE should be ~0 for exact synthetic data, was " + result.rmse,
                result.rmse < 1e-9);
    }

    @Test
    public void rejectsQuasistaticOnlyDataAsRankDeficient() {
        List<SvaFit.Sample> samples = new ArrayList<>();
        samples.add(sample(300, 0, "quasistatic"));
        samples.add(sample(600, 0, "quasistatic"));
        samples.add(sample(900, 0, "quasistatic"));
        samples.add(sample(1200, 0, "quasistatic"));
        samples.add(sample(1500, 0, "quasistatic"));

        SvaFit.FitResult result = SvaFit.fit(samples, true);

        assertFalse("Quasistatic-only data cannot identify kA and must be rejected",
                result.accepted);
    }

    @Test
    public void excludesRestSegmentSamplesSoKsStaysPositive() {
        // The kS bug: "fast-rest" samples (a=0, wheel braking, power
        // opposite the velocity) drag the fitted kS negative. They must
        // be excluded, and the fit must still recover the true constants.
        List<SvaFit.Sample> samples = goodSamples();
        // Braking-to-rest: v > 0 but power < 0 (controller braking).
        samples.add(new SvaFit.Sample(500, 0, -0.4, BATTERY_VOLTAGE, "rest"));
        samples.add(new SvaFit.Sample(200, 0, -0.2, BATTERY_VOLTAGE, "rest"));

        SvaFit.FitResult result = SvaFit.fit(samples, true);

        assertTrue("Fit should be accepted once rest samples are excluded: " + result.rejectReason,
                result.accepted);
        assertEquals(2, result.samplesExcluded);
        assertTrue("kS must stay positive, was " + result.kS, result.kS > 0.0);
        assertEquals(TRUE_KS, result.kS, TOLERANCE);
        assertEquals(TRUE_KV, result.kV, TOLERANCE);
        assertEquals(TRUE_KA, result.kA, TOLERANCE);
    }
}
