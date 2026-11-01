package org.firstinspires.ftc.teamcode.util.autotune;

/**
 * First-order low-pass filter for signed velocity.
 *
 * <p>Pure Java, no FTC dependency. {@code v[k] = alpha * raw[k] +
 * (1 - alpha) * v[k-1]}, {@code alpha = dt / (tau + dt)}.
 * Group delay is approximately {@code tau}; the default tau of
 * 0.04&nbsp;s is the design value from the SVA math report and must be
 * recorded in tuning provenance whenever it is used. The filter must
 * not be grown without re-checking relay timing, because its lag
 * corrupts the measured oscillation period.
 */
public final class VelocityFilter {
    private final double tauSeconds;
    private double filtered;
    private boolean initialised;

    public VelocityFilter(double tauSeconds) {
        if (!Double.isFinite(tauSeconds) || tauSeconds < 0.0) {
            throw new IllegalArgumentException("tauSeconds must be finite and >= 0.");
        }
        this.tauSeconds = tauSeconds;
    }

    /** Updates with a raw sample and returns the filtered value. Non-positive dt returns the previous value unchanged. */
    public double update(double raw, double dtSeconds) {
        if (!initialised) {
            filtered = raw;
            initialised = true;
            return filtered;
        }
        if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0) {
            return filtered;
        }
        double alpha = dtSeconds / (tauSeconds + dtSeconds);
        filtered += alpha * (raw - filtered);
        return filtered;
    }

    public double getFiltered() {
        return filtered;
    }

    public double getTauSeconds() {
        return tauSeconds;
    }

    public void reset() {
        initialised = false;
        filtered = 0.0;
    }
}
