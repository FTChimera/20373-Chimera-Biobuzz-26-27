package org.firstinspires.ftc.teamcode.util.control;

/**
 * Gains for {@link DoubleFlywheelFullVelocityController}.
 *
 * <p>Units (single source of truth; see the controller Javadoc):
 * state and target are ticks per second, time is seconds, output is
 * unitless motor power. {@code kP} is power per ticks/sec, {@code kI} is
 * power per (ticks/sec * s), {@code kD} is power per (ticks/sec^2),
 * {@code kS} is power, {@code kV} is power per ticks/sec, {@code kA} is
 * power per (ticks/sec^2), {@code tolerance} is ticks/sec, and
 * {@code integralLimit} is the cap on |integralSum| in ticks (ticks/sec * s).
 *
 * <p>Voltage compensation: {@code voltageCompensation} defaults to
 * {@code true} (decided 2026-11-01). When it is true, the controller
 * expects {@code kS}/{@code kV}/{@code kA} to be stored in <b>volts</b>
 * (kS in V, kV in V per ticks/sec, kA in V per ticks/sec^2) and divides
 * the feedforward voltage by the measured battery voltage each loop.
 * When it is false, {@code kS}/{@code kV}/{@code kA} are in power units
 * and are valid only near the battery voltage recorded during tuning.
 * {@code kP}/{@code kI}/{@code kD} are always in power units. The
 * controller must be given a voltage supplier for compensation to do
 * anything; with no supplier wired it is inert (scale 1.0), never an error.
 * {@code nominalVoltage} is used only for display/conversion, for the
 * no-reading fallback, and for sanity checks — never as the live divisor
 * when a measured reading exists.
 *
 * <p>DRAFT ONLY — not yet applied to any repository. The 8-argument
 * constructor and field order are preserved from HEAD 31b4a45. The
 * 10-argument constructor and the two voltage fields are the only
 * additions, justified by the follow-up decision that the constants
 * class default for voltage compensation must be enabled. Validation
 * is fail-fast: a NaN gain or a negative tolerance/limit can only be a
 * configuration error.
 */
public class DoubleFlywheelFullVelocityConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;

    public double tolerance;
    public double integralLimit;

    /** Voltage compensation flag. Default {@code true}. TODO-USER-CONSTANTS: confirm nominal voltage. */
    public boolean voltageCompensation;
    /** Nominal battery voltage in volts, used only as described above. Default 12.0. TODO-USER-CONSTANTS. */
    public double nominalVoltage;

    public DoubleFlywheelFullVelocityConstants(double p, double i, double d, double s, double v, double a, double tolerance, double integralLimit) {
        this(p, i, d, s, v, a, tolerance, integralLimit, true, 12.0);
    }

    public DoubleFlywheelFullVelocityConstants(double p, double i, double d, double s, double v, double a,
                                               double tolerance, double integralLimit,
                                               boolean voltageCompensation, double nominalVoltage) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kS = s;
        this.kV = v;
        this.kA = a;
        this.tolerance = tolerance;
        this.integralLimit = integralLimit;
        this.voltageCompensation = voltageCompensation;
        this.nominalVoltage = nominalVoltage;
        validate();
    }

    private void validate() {
        if (!Double.isFinite(kP) || !Double.isFinite(kI) || !Double.isFinite(kD)
                || !Double.isFinite(kS) || !Double.isFinite(kV) || !Double.isFinite(kA)
                || !Double.isFinite(tolerance) || !Double.isFinite(integralLimit)
                || !Double.isFinite(nominalVoltage)) {
            throw new IllegalArgumentException("DoubleFlywheelFullVelocityConstants: all gains, tolerance, integralLimit and nominalVoltage must be finite.");
        }
        if (tolerance < 0) {
            throw new IllegalArgumentException("DoubleFlywheelFullVelocityConstants: tolerance must be >= 0.");
        }
        if (integralLimit < 0) {
            throw new IllegalArgumentException("DoubleFlywheelFullVelocityConstants: integralLimit must be >= 0.");
        }
        if (nominalVoltage <= 0) {
            throw new IllegalArgumentException("DoubleFlywheelFullVelocityConstants: nominalVoltage must be > 0.");
        }
    }
}
