package org.firstinspires.ftc.teamcode.robotDev.sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Pure numerical checks shared by the three shooter/aim diagnostics. */
public final class ShooterAimMetrics {
    private ShooterAimMetrics() { }

    public static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    public static double wrappedError(double target, double actual) {
        if (!finite(target, actual)) return Double.NaN;
        return Math.atan2(Math.sin(target - actual), Math.cos(target - actual));
    }

    // Same inclusive wrap-safe check as MainTeleOp.headingReady, without its Dashboard class.
    public static boolean headingReady(double target, double actual, double toleranceDegrees) {
        if (!Double.isFinite(toleranceDegrees) || toleranceDegrees < 0 || toleranceDegrees > 180) {
            throw new IllegalArgumentException("Heading tolerance must be between 0 and 180 degrees.");
        }
        return withinHeadingError(wrappedError(target, actual), Math.toRadians(toleranceDegrees));
    }

    public static boolean withinHeadingError(double errorRadians, double toleranceRadians) {
        return finite(errorRadians, toleranceRadians) && toleranceRadians >= 0
                && toleranceRadians <= Math.PI && Math.abs(errorRadians) <= toleranceRadians;
    }

    public static boolean wheelsReady(double mainTarget, double mainActual,
                                      double counterTarget, double counterActual, double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0) {
            throw new IllegalArgumentException("Wheel tolerance must be finite and nonnegative.");
        }
        return finite(mainTarget, mainActual, counterTarget, counterActual)
                && mainTarget > 0 && counterTarget > 0
                && Math.abs(mainTarget - mainActual) <= tolerance
                && Math.abs(counterTarget - counterActual) <= tolerance;
    }

    /** Y-only arbitration: SMART on both this and the previous loop, no overrides/manual feed. */
    public static boolean yAutoFire(boolean previousSmart, boolean wheelGate, boolean headingGate) {
        return previousSmart && wheelGate && headingGate;
    }

    public static final class Dwell {
        public static final double MAX_SAMPLE_GAP_S = 0.1;
        private double since = Double.NaN;
        private double last = Double.NaN;
        public double firstEntry = Double.NaN;
        public double settledAt = Double.NaN;
        public double longest = 0;
        public double current = 0;

        public boolean sample(double elapsed, boolean ready, double requiredSeconds) {
            if (!Double.isFinite(requiredSeconds) || requiredSeconds <= 0) {
                throw new IllegalArgumentException("Dwell must be finite and positive.");
            }
            if (!Double.isFinite(elapsed) || elapsed < 0 || (Double.isFinite(last) && elapsed < last)) {
                since = Double.NaN;
                current = 0;
                last = Double.NaN;
                return false;
            }
            if (Double.isFinite(last) && elapsed - last > MAX_SAMPLE_GAP_S) {
                since = Double.NaN;
                current = 0;
            }
            last = elapsed;
            if (!ready) {
                since = Double.NaN;
                current = 0;
                return false;
            }
            if (Double.isNaN(firstEntry)) firstEntry = elapsed;
            if (Double.isNaN(since)) since = elapsed;
            current = elapsed - since;
            longest = Math.max(longest, current);
            boolean settled = current >= requiredSeconds;
            if (settled && Double.isNaN(settledAt)) settledAt = elapsed;
            return settled;
        }
    }

    public static final class Heading {
        private final List<Double> errors = new ArrayList<>();
        private double sumSquares;
        public double maxAbs;
        public boolean valid = true;

        public void sample(double target, double actual) {
            double error = wrappedError(target, actual);
            if (!Double.isFinite(error)) {
                valid = false;
                return;
            }
            errors.add(error);
            maxAbs = Math.max(maxAbs, Math.abs(error));
            sumSquares += error * error;
        }

        public int count() { return errors.size(); }
        public double rms() { return count() == 0 ? Double.NaN : Math.sqrt(sumSquares / count()); }

        /** Smallest circular arc containing every sampled error, not a naive +/-180 range. */
        public double peakToPeak() {
            if (!valid || errors.isEmpty()) return Double.NaN;
            List<Double> sorted = new ArrayList<>(errors);
            Collections.sort(sorted);
            double largestGap = sorted.get(0) + 2 * Math.PI - sorted.get(sorted.size() - 1);
            for (int i = 1; i < sorted.size(); i++) {
                largestGap = Math.max(largestGap, sorted.get(i) - sorted.get(i - 1));
            }
            return Math.max(0, 2 * Math.PI - largestGap);
        }

        public boolean within(double toleranceDegrees) {
            return valid && count() > 0 && Double.isFinite(toleranceDegrees)
                    && withinHeadingError(maxAbs, Math.toRadians(toleranceDegrees));
        }
    }

    public static boolean movingPass(Heading moving, double displacement, int minimumSamples,
                                     double minimumDisplacement, double headingToleranceDegrees) {
        return finite(displacement, minimumDisplacement) && minimumDisplacement > 0
                && minimumSamples > 0 && moving.count() >= minimumSamples
                && displacement >= minimumDisplacement && moving.within(headingToleranceDegrees);
    }

    public static boolean holdPass(Heading heading, double duration, double requiredDuration,
                                   double drift, double maxSpeed, double driftLimit, double speedLimit,
                                   double toleranceDegrees) {
        return finite(duration, requiredDuration, drift, maxSpeed, driftLimit, speedLimit)
                && requiredDuration > 0 && duration >= requiredDuration
                && drift >= 0 && drift <= driftLimit && maxSpeed >= 0 && maxSpeed <= speedLimit
                && heading.within(toleranceDegrees);
    }

    public static boolean windowCovered(double lastSample, double boundary, double maxGap,
                                        double duration, double allowedGap) {
        return finite(lastSample, boundary, maxGap, duration, allowedGap)
                && duration > 0 && allowedGap > 0 && allowedGap < duration
                && lastSample >= duration - allowedGap && lastSample <= duration
                && boundary >= duration && boundary - lastSample <= allowedGap
                && maxGap >= 0 && maxGap <= allowedGap;
    }
}
