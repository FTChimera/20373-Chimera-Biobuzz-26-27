package org.firstinspires.ftc.teamcode.util;

/**
 * Detects whether TeamCode is running under the ftcsim desktop simulator
 * (TeamCodeSim) instead of on the robot's Control Hub.
 *
 * <p>Resolution order in {@link #isSimulation()}:
 * <ol>
 *   <li>The {@link #override} — when non-null it wins. This is the fallback /
 *   testing hook: unit tests (and only unit tests) may force sim or
 *   real-hardware behaviour through {@link #setOverride(Boolean)}.
 *   Production robot code must never set it.</li>
 *   <li>The {@code ftcsim.jvm} system property, set to {@code "true"} by
 *   TeamCodeSim's {@code run} task ({@code TeamCodeSim/build.gradle}).</li>
 *   <li>Whether the {@code ftcsim.jvm.DesktopSdk} class is loadable — it is
 *   only on the simulator classpath, never on the robot.</li>
 * </ol>
 *
 * <p>Robot's constructor uses this to swap in sim-only fakes (stub Vision,
 * stub Lights) for hardware ftcsim cannot provide. When this returns false,
 * production behaviour is unchanged.
 */
public final class SimEnvironment {
    private SimEnvironment() {}

    /**
     * Forced answer for {@link #isSimulation()}, or null for auto-detection.
     * Test hook only — never set from production robot code. Prefer
     * {@link #setOverride(Boolean)} so the intent is explicit at the call site.
     */
    public static volatile Boolean override = null;

    /** Sets {@link #override}; pass null to restore auto-detection. Test hook only. */
    public static void setOverride(Boolean value) {
        override = value;
    }

    /** True when TeamCode is running under the ftcsim desktop simulator. */
    public static boolean isSimulation() {
        Boolean forced = override;
        if (forced != null) {
            return forced;
        }
        if (Boolean.getBoolean("ftcsim.jvm")) {
            return true;
        }
        try {
            Class.forName("ftcsim.jvm.DesktopSdk");
            return true;
        } catch (Throwable ignored) {
            return false;
        }
    }
}
