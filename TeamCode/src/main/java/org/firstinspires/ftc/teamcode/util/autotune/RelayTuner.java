package org.firstinspires.ftc.teamcode.util.autotune;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Corrected relay-feedback tuner (Astrom-Hagglund) for a velocity loop.
 *
 * <p>Pure Java, no FTC dependency. The caller feeds signed velocity
 * samples with monotonic timestamps in seconds and applies the
 * returned power. This class deliberately does <b>not</b> reuse
 * Aarav's {@code RelayAutoTuner.computeResult()} amplitude: that
 * implementation tracks only the maximum while the relay is high and
 * only the minimum while it is low, so it records threshold-region
 * values instead of turning points and overestimates Ku by roughly
 * 2x-10x on the replica inputs checked in the math audit (finding F1).
 * Here every interval records <b>both</b> its minimum and maximum, and
 * each cycle's peak/trough are the turning points over two consecutive
 * intervals.
 *
 * <p>Consistency gates are applied at {@link #computeResult()}: CV
 * checks on period and amplitude, outlier-cycle discards (at most 2),
 * half-period asymmetry and mean-offset warnings, and explicit
 * reversal / overspeed / chatter / no-progress failure detection.
 * A failed run issues no Ku/Tu.
 */
public final class RelayTuner {

    /** Named failure outcome; {@link #NONE} means no failure detected. */
    public enum Failure {
        NONE,
        NO_OSCILLATION,
        CHATTER,
        REVERSAL,
        OVERSPEED,
        TIMEOUT,
        INCONSISTENT,
        INSUFFICIENT_CYCLES
    }

    /** One usable relay cycle, after discarding ignored cycles. */
    public static final class Cycle {
        public final double periodSeconds;
        public final double amplitudeTicksPerSec;
        public final double meanTicksPerSec;
        public final double highDurationSeconds;
        public final double lowDurationSeconds;

        Cycle(double periodSeconds, double amplitudeTicksPerSec, double meanTicksPerSec,
              double highDurationSeconds, double lowDurationSeconds) {
            this.periodSeconds = periodSeconds;
            this.amplitudeTicksPerSec = amplitudeTicksPerSec;
            this.meanTicksPerSec = meanTicksPerSec;
            this.highDurationSeconds = highDurationSeconds;
            this.lowDurationSeconds = lowDurationSeconds;
        }
    }

    /** Result of a relay run. {@code success} is false when no Ku/Tu may be used. */
    public static final class Result {
        public final boolean success;
        public final Failure failure;
        public final String failureDetail;
        public final double ku;
        public final double tuSeconds;
        public final double amplitudeTicksPerSec;
        public final double periodCv;
        public final double amplitudeCv;
        public final List<Cycle> cycles;
        public final List<String> warnings;
        public final int switchCount;
        public final int discardedCycleCount;

        Result(boolean success, Failure failure, String failureDetail,
               double ku, double tuSeconds, double amplitudeTicksPerSec,
               double periodCv, double amplitudeCv,
               List<Cycle> cycles, List<String> warnings,
               int switchCount, int discardedCycleCount) {
            this.success = success;
            this.failure = failure;
            this.failureDetail = failureDetail;
            this.ku = ku;
            this.tuSeconds = tuSeconds;
            this.amplitudeTicksPerSec = amplitudeTicksPerSec;
            this.periodCv = periodCv;
            this.amplitudeCv = amplitudeCv;
            this.cycles = cycles;
            this.warnings = warnings;
            this.switchCount = switchCount;
            this.discardedCycleCount = discardedCycleCount;
        }
    }

    private static final class Interval {
        final double startSeconds;
        final double endSeconds;
        final double minVelocity;
        final double maxVelocity;
        final boolean outputHighDuring;

        Interval(double startSeconds, double endSeconds, double minVelocity, double maxVelocity, boolean outputHighDuring) {
            this.startSeconds = startSeconds;
            this.endSeconds = endSeconds;
            this.minVelocity = minVelocity;
            this.maxVelocity = maxVelocity;
            this.outputHighDuring = outputHighDuring;
        }

        double durationSeconds() {
            return endSeconds - startSeconds;
        }
    }

    private final double targetTicksPerSec;
    private final double relayAmplitude;
    private final double hysteresisTicksPerSec;
    private final int cyclesToCollect;
    private final int cyclesToIgnore;
    private final double timeoutSeconds;
    private final double noProgressSeconds;

    private boolean outputHigh = true;
    private double intervalStartSeconds = Double.NaN;
    private double intervalMin = Double.POSITIVE_INFINITY;
    private double intervalMax = Double.NEGATIVE_INFINITY;
    private double lastSwitchSeconds = Double.NaN;
    private double firstSampleSeconds = Double.NaN;
    private double lastSampleSeconds = Double.NaN;
    private final List<Interval> intervals = new ArrayList<>();
    private final List<Double> switchTimes = new ArrayList<>();
    private int consecutiveNegativeSamples;
    private Failure failure = Failure.NONE;
    private String failureDetail = "";

    /**
     * @param targetTicksPerSec target velocity, must be &gt; 0 (zero/negative targets are rejected).
     * @param relayAmplitude relay output magnitude in power units, in (0, 1].
     * @param hysteresisTicksPerSec relay deadband in ticks/sec, must be &gt; 0.
     * @param cyclesToCollect usable cycles to average, must be &gt;= 4.
     * @param cyclesToIgnore initial cycles to discard, must be &gt;= 1 in the design (0 accepted here only to keep the class testable; callers should pass &gt;= 1).
     * @param timeoutSeconds elapsed-time limit for the phase, must be &gt; 0.
     */
    public RelayTuner(double targetTicksPerSec, double relayAmplitude, double hysteresisTicksPerSec,
                      int cyclesToCollect, int cyclesToIgnore,
                      double timeoutSeconds, double noProgressSeconds) {
        if (!Double.isFinite(targetTicksPerSec) || targetTicksPerSec <= 0.0) {
            throw new IllegalArgumentException("Relay target must be finite and > 0.");
        }
        if (!Double.isFinite(relayAmplitude) || relayAmplitude <= 0.0 || relayAmplitude > 1.0) {
            throw new IllegalArgumentException("Relay amplitude must be in (0, 1].");
        }
        if (!Double.isFinite(hysteresisTicksPerSec) || hysteresisTicksPerSec <= 0.0) {
            throw new IllegalArgumentException("Hysteresis must be finite and > 0.");
        }
        if (cyclesToCollect < 1) {
            throw new IllegalArgumentException("cyclesToCollect must be >= 1.");
        }
        if (cyclesToIgnore < 0) {
            throw new IllegalArgumentException("cyclesToIgnore must be >= 0.");
        }
        if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
            throw new IllegalArgumentException("timeoutSeconds must be finite and > 0.");
        }
        if (!Double.isFinite(noProgressSeconds) || noProgressSeconds <= 0.0) {
            throw new IllegalArgumentException("noProgressSeconds must be finite and > 0.");
        }
        this.targetTicksPerSec = targetTicksPerSec;
        this.relayAmplitude = relayAmplitude;
        this.hysteresisTicksPerSec = hysteresisTicksPerSec;
        this.cyclesToCollect = cyclesToCollect;
        this.cyclesToIgnore = cyclesToIgnore;
        this.timeoutSeconds = timeoutSeconds;
        this.noProgressSeconds = noProgressSeconds;
    }

    /**
     * Feeds one signed velocity sample and returns the relay power to
     * apply ({@code +d} or {@code -d}, or {@code 0} once a failure has
     * been detected). Timestamps must be monotonic seconds.
     */
    public double update(double velocityTicksPerSec, double timestampSeconds) {
        if (failure != Failure.NONE) {
            return 0.0;
        }
        if (!Double.isFinite(velocityTicksPerSec) || !Double.isFinite(timestampSeconds)) {
            return outputHigh ? relayAmplitude : -relayAmplitude;
        }
        if (Double.isNaN(firstSampleSeconds)) {
            firstSampleSeconds = timestampSeconds;
            intervalStartSeconds = timestampSeconds;
        }
        lastSampleSeconds = timestampSeconds;

        // Elapsed timeout (phase-start based; never absolute runtime).
        if (timestampSeconds - firstSampleSeconds > timeoutSeconds) {
            fail(Failure.TIMEOUT, "Relay timeout after " + timeoutSeconds + " s elapsed with " + switchTimes.size() + " switches.");
            return 0.0;
        }
        // Overspeed: target + max(300 ticks/s, 20% of target).
        double overspeedLimit = targetTicksPerSec + Math.max(300.0, 0.20 * targetTicksPerSec);
        if (velocityTicksPerSec > overspeedLimit) {
            fail(Failure.OVERSPEED, "Velocity " + velocityTicksPerSec + " exceeded overspeed limit " + overspeedLimit + ".");
            return 0.0;
        }
        // Reversal: signed velocity below zero for >3 consecutive samples.
        if (velocityTicksPerSec < 0.0) {
            consecutiveNegativeSamples++;
            if (consecutiveNegativeSamples > 3) {
                fail(Failure.REVERSAL, "Wheel reversed for more than 3 consecutive samples; raise target or lower relay amplitude.");
                return 0.0;
            }
        } else {
            consecutiveNegativeSamples = 0;
        }

        intervalMin = Math.min(intervalMin, velocityTicksPerSec);
        intervalMax = Math.max(intervalMax, velocityTicksPerSec);

        boolean switched = false;
        if (outputHigh && velocityTicksPerSec > targetTicksPerSec + hysteresisTicksPerSec) {
            closeInterval(timestampSeconds);
            outputHigh = false;
            switched = true;
        } else if (!outputHigh && velocityTicksPerSec < targetTicksPerSec - hysteresisTicksPerSec) {
            closeInterval(timestampSeconds);
            outputHigh = true;
            switched = true;
        }
        if (switched) {
            switchTimes.add(timestampSeconds);
            // Chatter: more than 6 switches in any 0.5 s window.
            int recent = 0;
            for (int i = switchTimes.size() - 1; i >= 0; i--) {
                if (timestampSeconds - switchTimes.get(i) <= 0.5) {
                    recent++;
                } else {
                    break;
                }
            }
            if (recent > 6) {
                fail(Failure.CHATTER, "More than 6 relay switches in 0.5 s; increase hysteresis.");
                return 0.0;
            }
        } else if (!switchTimes.isEmpty()
                && timestampSeconds - lastSwitchSeconds > noProgressSeconds) {
            fail(Failure.NO_OSCILLATION, "No relay switch for " + noProgressSeconds + " s after the first switch.");
            return 0.0;
        }
        return outputHigh ? relayAmplitude : -relayAmplitude;
    }

    private void closeInterval(double timestampSeconds) {
        intervals.add(new Interval(intervalStartSeconds, timestampSeconds, intervalMin, intervalMax, outputHigh));
        lastSwitchSeconds = timestampSeconds;
        intervalStartSeconds = timestampSeconds;
        intervalMin = Double.POSITIVE_INFINITY;
        intervalMax = Double.NEGATIVE_INFINITY;
    }

    private void fail(Failure failure, String detail) {
        this.failure = failure;
        this.failureDetail = detail;
    }

    /** True once enough closed intervals exist for the requested usable cycles. */
    public boolean isFinished() {
        return intervals.size() >= (cyclesToCollect + cyclesToIgnore) * 2;
    }

    public Failure getFailure() {
        return failure;
    }

    public String getFailureDetail() {
        return failureDetail;
    }

    public int getSwitchCount() {
        return switchTimes.size();
    }

    public double getTargetTicksPerSec() {
        return targetTicksPerSec;
    }

    public double getRelayAmplitude() {
        return relayAmplitude;
    }

    /**
     * Computes Ku/Tu from closed intervals, applying the consistency
     * gates. Call after {@link #isFinished()}, on stop-request with
     * enough cycles, or on timeout. Never issues Ku/Tu on failure.
     */
    public Result computeResult() {
        List<String> warnings = new ArrayList<>();
        if (failure == Failure.REVERSAL || failure == Failure.OVERSPEED
                || failure == Failure.CHATTER || failure == Failure.TIMEOUT) {
            return emptyResult(warnings);
        }
        // Build cycles from consecutive interval pairs.
        List<Cycle> allCycles = new ArrayList<>();
        for (int i = 0; i + 1 < intervals.size(); i += 2) {
            Interval a = intervals.get(i);
            Interval b = intervals.get(i + 1);
            double period = a.durationSeconds() + b.durationSeconds();
            if (period <= 0.0) {
                continue;
            }
            double peak = Math.max(a.maxVelocity, b.maxVelocity);
            double trough = Math.min(a.minVelocity, b.minVelocity);
            double highDuration = a.outputHighDuring ? a.durationSeconds() : b.durationSeconds();
            double lowDuration = period - highDuration;
            allCycles.add(new Cycle(period, (peak - trough) / 2.0, (peak + trough) / 2.0, highDuration, lowDuration));
        }
        if (allCycles.size() <= cyclesToIgnore) {
            return new Result(false, Failure.INSUFFICIENT_CYCLES,
                    "Only " + allCycles.size() + " cycles closed, ignoring " + cyclesToIgnore + "; no usable cycles remain.",
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    Collections.emptyList(), warnings, switchTimes.size(), 0);
        }
        List<Cycle> usable = new ArrayList<>(allCycles.subList(cyclesToIgnore, allCycles.size()));

        // Outlier discard: |Tu_i - median(Tu)| / median(Tu) > 0.30, at most 2
        // discards; more than 2 outliers rejects the run as inconsistent.
        List<Cycle> kept = new ArrayList<>(usable);
        int discarded = 0;
        if (!kept.isEmpty()) {
            double medianTu = median(periods(kept));
            int outlierCount = 0;
            for (Cycle c : kept) {
                if (medianTu > 0.0 && Math.abs(c.periodSeconds - medianTu) / medianTu > 0.30) {
                    outlierCount++;
                }
            }
            if (outlierCount > 2) {
                return new Result(false, Failure.INCONSISTENT,
                        "More than 2 outlier cycles (" + outlierCount + "); relay data are inconsistent.",
                        Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                        Collections.emptyList(), warnings, switchTimes.size(), 0);
            }
            List<Cycle> survivors = new ArrayList<>();
            for (Cycle c : kept) {
                if (medianTu > 0.0 && Math.abs(c.periodSeconds - medianTu) / medianTu > 0.30) {
                    discarded++;
                    continue;
                }
                survivors.add(c);
            }
            kept = survivors;
        }
        if (kept.size() < 4) {
            return new Result(false, Failure.INSUFFICIENT_CYCLES,
                    "Only " + kept.size() + " usable cycles remain after discards; need at least 4.",
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    Collections.emptyList(), warnings, switchTimes.size(), discarded);
        }
        if (kept.size() < cyclesToCollect) {
            warnings.add("Only " + kept.size() + " usable cycles (requested " + cyclesToCollect + "); Ku/Tu are averaged over the cycles available.");
        }

        double tu = mean(periods(kept));
        double amplitude = mean(amplitudes(kept));
        double periodCv = coefficientOfVariation(periods(kept));
        double amplitudeCv = coefficientOfVariation(amplitudes(kept));
        if (amplitude <= 0.0 || tu <= 0.0) {
            return new Result(false, Failure.NO_OSCILLATION,
                    "Non-positive period or amplitude; no oscillation was measured.",
                    Double.NaN, Double.NaN, Double.NaN, periodCv, amplitudeCv,
                    Collections.emptyList(), warnings, switchTimes.size(), discarded);
        }
        if (periodCv > 0.20) {
            return new Result(false, Failure.INCONSISTENT,
                    "Period CV " + periodCv + " exceeds 0.20; relay data are inconsistent.",
                    Double.NaN, Double.NaN, Double.NaN, periodCv, amplitudeCv,
                    Collections.emptyList(), warnings, switchTimes.size(), discarded);
        }
        if (amplitudeCv > 0.25) {
            return new Result(false, Failure.INCONSISTENT,
                    "Amplitude CV " + amplitudeCv + " exceeds 0.25; relay data are inconsistent.",
                    Double.NaN, Double.NaN, Double.NaN, periodCv, amplitudeCv,
                    Collections.emptyList(), warnings, switchTimes.size(), discarded);
        }
        if (periodCv > 0.10) {
            warnings.add("Period CV " + periodCv + " is elevated (warn band 0.10-0.20).");
        }
        if (amplitudeCv > 0.15) {
            warnings.add("Amplitude CV " + amplitudeCv + " is elevated (warn band 0.15-0.25).");
        }
        double meanAsymmetry = 0.0;
        for (Cycle c : kept) {
            meanAsymmetry += Math.abs(c.highDurationSeconds - c.lowDurationSeconds) / c.periodSeconds;
        }
        meanAsymmetry /= kept.size();
        if (meanAsymmetry > 0.40) {
            warnings.add("Mean half-period asymmetry " + meanAsymmetry + " exceeds 0.40; limit cycle is asymmetric.");
        }
        double meanOfMeans = mean(means(kept));
        if (Math.abs(meanOfMeans - targetTicksPerSec) / amplitude > 0.30) {
            warnings.add("Mean cycle offset |mean-target|/a = "
                    + (Math.abs(meanOfMeans - targetTicksPerSec) / amplitude)
                    + " exceeds 0.30; Ku formula assumes a symmetric limit cycle.");
        }
        double ku = 4.0 * relayAmplitude / (Math.PI * amplitude);
        return new Result(true, Failure.NONE, "", ku, tu, amplitude,
                periodCv, amplitudeCv, Collections.unmodifiableList(kept),
                warnings, switchTimes.size(), discarded);
    }

    private Result emptyResult(List<String> warnings) {
        return new Result(false, failure, failureDetail,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Collections.emptyList(), warnings, switchTimes.size(), 0);
    }

    private static List<Double> periods(List<Cycle> cycles) {
        List<Double> out = new ArrayList<>(cycles.size());
        for (Cycle c : cycles) {
            out.add(c.periodSeconds);
        }
        return out;
    }

    private static List<Double> amplitudes(List<Cycle> cycles) {
        List<Double> out = new ArrayList<>(cycles.size());
        for (Cycle c : cycles) {
            out.add(c.amplitudeTicksPerSec);
        }
        return out;
    }

    private static List<Double> means(List<Cycle> cycles) {
        List<Double> out = new ArrayList<>(cycles.size());
        for (Cycle c : cycles) {
            out.add(c.meanTicksPerSec);
        }
        return out;
    }

    private static double mean(List<Double> values) {
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return values.isEmpty() ? Double.NaN : sum / values.size();
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int n = sorted.size();
        if (n == 0) {
            return Double.NaN;
        }
        if (n % 2 == 1) {
            return sorted.get(n / 2);
        }
        return (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0;
    }

    private static double coefficientOfVariation(List<Double> values) {
        if (values.isEmpty()) {
            return Double.NaN;
        }
        double m = mean(values);
        if (m == 0.0) {
            return Double.NaN;
        }
        double sumSq = 0.0;
        for (double v : values) {
            sumSq += (v - m) * (v - m);
        }
        return Math.sqrt(sumSq / values.size()) / Math.abs(m);
    }
}
