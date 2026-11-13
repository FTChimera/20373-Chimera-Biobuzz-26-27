package org.firstinspires.ftc.teamcode.util.control;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * PIDSVA full-velocity controller for a single flywheel.
 *
 * <p>Units (single source of truth): state and target are <b>ticks per
 * second</b>, time is <b>seconds</b>, output is unitless motor power in
 * {@code [-1, 1]}. Gains: {@code kP} power per ticks/sec, {@code kI}
 * power per (ticks/sec * s), {@code kD} power per (ticks/sec^2),
 * {@code kS} power (or volts — see voltage compensation), {@code kV}
 * power per ticks/sec, {@code kA} power per (ticks/sec^2).
 * {@code tolerance} is ticks/sec, {@code integralLimit} is the cap on
 * |integralSum| in ticks. RPM is a tuner/UI unit only and must be
 * converted to ticks/sec at the tuner boundary, never here.
 *
 * <p>Control law per update:
 * <pre>
 *   feedforward = sgn(target) * |kS| + kV * target + kA * targetAcceleration
 *       (feedforward is voltage-scaled when compensation is active; see below)
 *   feedback    = kP * error + kI * integralSum + kD * filteredDerivative
 *       (error = target - state; derivative is of error, first-order
 *        low-pass filtered; integral is clamped and frozen on saturation)
 *   output      = clamp(feedforward + feedback, -1, 1)
 * </pre>
 * {@code targetAcceleration} is {@code (target - lastTarget) / dt}, i.e.
 * the rate of change of the velocity setpoint. It is exactly zero for a
 * constant setpoint, so {@code kA} contributes only during setpoint
 * changes/ramps. Setpoint steps therefore spike both the derivative and
 * the kA term; callers should ramp setpoints where possible.
 *
 * <p>Zero-target deadband: when {@code target == 0} and
 * {@code |state| <= tolerance}, the output is forced to 0 and the
 * integral/derivative state is cleared, instead of servoing to exactly
 * zero. The feedback gains are tuned at speed (Ziegler-Nichols), where
 * the velocity signal dwarfs encoder noise; at standstill the same
 * gains amplify that noise into full-scale power oscillation (the
 * derivative term dominates). The wheel still brakes normally from
 * speed; the deadband only engages once it is nearly stopped.
 *
 * <p>Voltage compensation (default ON, per the 2026-11-01 decision):
 * when {@link DoubleFlywheelFullVelocityConstants#voltageCompensation}
 * is true, the stored {@code kS}/{@code kV}/{@code kA} are in
 * <b>volts</b> and the feedforward voltage is <b>divided by the
 * measured battery voltage</b> once per update, using a fresh reading
 * from the supplier set with {@link #setVoltageSupplier(DoubleSupplier)}
 * (so {@code u_ff = V_ff / Vbatt}, matching the SVA fit, which stores
 * per-sample {@code power * Vbatt}). Feedback terms are <b>not</b>
 * divided: feedback is closed-loop error correction, and scaling it
 * would make loop gain battery-dependent; feedforward is an open-loop
 * voltage prediction. When the flag is true but no supplier is set,
 * the reading is non-finite, outside {@code [0.75 * nominal,
 * 1.20 * nominal]} (implausible for the battery), or the supplier
 * throws, the divisor for that cycle is {@code nominalVoltage} (the
 * no-reading fallback) — never a stale reading, never an exception,
 * and {@code nominalVoltage} is never the live divisor when a valid
 * measured reading exists. When the flag is false, the stored
 * {@code kS}/{@code kV}/{@code kA} are in power units and the
 * feedforward is used as-is (scale 1.0), valid only near the battery
 * voltage recorded during tuning. The scale actually applied is
 * visible via {@link #getAppliedVoltageScale()}.
 *
 * <p>CONVENTION FIX (coordinator, draft stage): the first draft of
 * this file multiplied the feedforward by
 * {@code nominalVoltage / measuredVoltage}. That is only correct if
 * the stored constants are power-at-nominal-voltage, which
 * contradicts the constants Javadoc and the SVA fit (both volts).
 * This file was corrected to the volts/divide convention so the
 * tuner, the fit, and the controller agree. The auditor's FR-1.6
 * {@code nominal/measured} scale is superseded; see the coordinator
 * report for the evidence.
 *
 * <p>Threading: this controller assumes a single calling thread (the
 * OpMode loop). It is not synchronised. Callers must not mutate the
 * constants object's fields mid-loop. {@code getOutput()} before the
 * first update returns {@code 0.0}; {@code atTarget()} returns
 * {@code false} until the first update.
 *
 * <p>DRAFT ONLY — corrected in place from HEAD 31b4a45 (FR-1). Fixes,
 * each verified against the original file: double-seconds {@code dt}
 * (was {@code TimeUnit.toSeconds} truncation to whole seconds);
 * {@code reset()} restores the {@code lastTime = -1} sentinel (was
 * {@code 0}, making the sentinel dead and the first {@code dt} huge);
 * kS is zero at target zero (was {@code +|kS|} at stop); output
 * clamping; derivative low-pass filtering; saturation-aware integral
 * anti-windup; {@code integralLimit == 0} documented as integral
 * disabled; {@code atTarget()} false until first update; null and
 * non-finite guards. See the coordinator's fix record FR-1 for
 * before/after behaviour and on-robot verification still needed.
 */
public class DoubleFlywheelFullVelocityController implements Controller<Double, Double, Double> {
    /** Filter time constant for the derivative term, seconds. Internal tuning state, not a public gain. TODO-USER-CONSTANTS: confirm on robot. */
    static final double DERIVATIVE_FILTER_TAU_SECONDS = 0.02;

    private final DoubleFlywheelFullVelocityConstants constants;
    private DoubleSupplier voltageSupplier;
    private double appliedVoltageScale = 1.0;

    private double lastError;
    private double integralSum;
    private double filteredDerivative;
    private boolean hasUpdate;
    private long lastTime = -1;
    private double target;
    private double lastTarget;
    private double output;

    public DoubleFlywheelFullVelocityController(DoubleFlywheelFullVelocityConstants constants) {
        this.constants = Objects.requireNonNull(constants, "constants must not be null");
        reset();
    }

    /** Returns the constants object. The object itself remains mutable by shape preservation; do not mutate it mid-loop. */
    public DoubleFlywheelFullVelocityConstants getConstants() {
        return constants;
    }

    /**
     * Sets the battery-voltage source used for voltage compensation, e.g.
     * {@code () -> readBatteryVoltage(hardwareMap)}. May be null to clear
     * the supplier (compensation then stays inert at scale 1.0).
     */
    public void setVoltageSupplier(DoubleSupplier voltageSupplier) {
        this.voltageSupplier = voltageSupplier;
    }

    /** Scale applied to the feedforward terms on the most recent update: {@code 1 / Vbatt} when compensation is active with a valid reading, {@code 1 / nominalVoltage} on the no-reading fallback, and {@code 1.0} when compensation is off (power-unit constants). */
    public double getAppliedVoltageScale() {
        return appliedVoltageScale;
    }

    /**
     * Timed update. {@code dt} is in seconds. Non-finite or negative
     * {@code dt} is treated as 0 for this cycle. Non-finite state or
     * target leaves the previous output unchanged for this cycle.
     */
    public void update(Double state, double dt) {
        Objects.requireNonNull(state, "state must not be null");
        if (!Double.isFinite(state) || !Double.isFinite(target)) {
            return;
        }
        boolean usableDt = Double.isFinite(dt) && dt >= 0.0;
        double effectiveDt = usableDt ? dt : 0.0;
        boolean isFirstUpdate = !hasUpdate;

        double error = target - state;

        // Zero-target deadband: when commanded to stop and the wheel is
        // already within tolerance of stopped, cut power instead of
        // servoing to exactly zero. Without this, the feedback terms
        // (kP/kD from Ziegler-Nichols, tuned at speed) amplify encoder
        // velocity noise into full-scale power oscillation at standstill
        // (verified by simulation: RMS output ~0.73, peak +/-1.0, 116
        // sign flips / 200 steps at target 0 with modest noise, the
        // derivative term dominating). The wheel still brakes normally
        // from speed (|state| > tolerance); the deadband only engages
        // once it is nearly stopped, where coasting the last few
        // ticks/sec to zero is harmless. Integral and derivative state
        // are cleared so no stale term kicks in when a nonzero target
        // resumes.
        if (target == 0.0 && Math.abs(state) <= constants.tolerance) {
            output = 0.0;
            integralSum = 0.0;
            filteredDerivative = 0.0;
            lastTarget = target;
            lastError = error;
            hasUpdate = true;
            return;
        }

        // Feedforward (voltage-scaled) + feedback (unscaled), see class Javadoc.
        double scale = computeFeedforwardScale();
        appliedVoltageScale = scale;

        double feedforward = 0.0;
        if (target != 0.0) {
            // Static friction follows the commanded direction, never the
            // measured direction (which would chatter at standstill).
            feedforward += Math.signum(target) * Math.abs(constants.kS) * scale;
        }
        feedforward += constants.kV * target * scale;

        // Setpoint acceleration: rate of change of the velocity setpoint.
        double targetAcceleration = 0.0;
        if (!isFirstUpdate && effectiveDt > 0.0) {
            targetAcceleration = (target - lastTarget) / effectiveDt;
        }
        feedforward += constants.kA * targetAcceleration * scale;

        // Integral, with clamping and saturation-aware anti-windup.
        // The pre-cycle sum is kept so accumulation can be discarded if
        // this cycle's output saturates in the direction of the error.
        double integralBefore = integralSum;
        if (!isFirstUpdate && effectiveDt > 0.0) {
            integralSum += error * effectiveDt;
            if (Math.abs(integralSum) > constants.integralLimit) {
                integralSum = Math.copySign(constants.integralLimit, integralSum);
            }
        }

        // Derivative of error, first-order low-pass filtered.
        double rawDerivative = 0.0;
        if (!isFirstUpdate && effectiveDt > 0.0) {
            rawDerivative = (error - lastError) / effectiveDt;
        }
        if (isFirstUpdate) {
            filteredDerivative = 0.0;
        } else if (effectiveDt > 0.0) {
            double alpha = effectiveDt / (DERIVATIVE_FILTER_TAU_SECONDS + effectiveDt);
            filteredDerivative += alpha * (rawDerivative - filteredDerivative);
        }

        double feedback = constants.kP * error
                + constants.kI * integralSum
                + constants.kD * filteredDerivative;

        double unsaturated = feedforward + feedback;
        double clamped = Math.max(-1.0, Math.min(1.0, unsaturated));
        if (clamped != unsaturated && Math.signum(error) == Math.signum(clamped) && clamped != 0.0) {
            // Saturated and the error would push further the same way:
            // discard this cycle's integral accumulation.
            integralSum = integralBefore;
            feedback = constants.kP * error
                    + constants.kI * integralSum
                    + constants.kD * filteredDerivative;
            clamped = Math.max(-1.0, Math.min(1.0, feedforward + feedback));
        }
        output = clamped;

        lastTarget = target;
        lastError = error;
        hasUpdate = true;
    }

    @Override
    public void update(Double state) {
        long now = System.nanoTime();
        double dt;
        if (lastTime == -1) {
            // First call (or just after reset): baseline only, no elapsed time.
            dt = 0.0;
        } else {
            dt = (now - lastTime) / 1e9;
        }
        lastTime = now;
        update(state, dt);
    }

    /**
     * Scale applied to the feedforward terms. Compensation on: stored
     * constants are volts, so the scale is {@code 1 / divisor} where
     * the divisor is the fresh measured voltage when valid, else the
     * nominal voltage (no-reading fallback). Compensation off: 1.0
     * (constants are already in power units).
     */
    private double computeFeedforwardScale() {
        if (!constants.voltageCompensation) {
            return 1.0;
        }
        double divisor = constants.nominalVoltage;
        if (voltageSupplier != null) {
            double measured;
            try {
                measured = voltageSupplier.getAsDouble();
            } catch (RuntimeException e) {
                measured = Double.NaN;
            }
            double lo = 0.75 * constants.nominalVoltage;
            double hi = 1.20 * constants.nominalVoltage;
            if (Double.isFinite(measured) && measured >= lo && measured <= hi) {
                divisor = measured;
            }
        }
        return 1.0 / divisor;
    }

    public double lastError() {
        return lastError;
    }

    @Override
    public void reset() {
        hasUpdate = false;
        integralSum = 0.0;
        lastError = 0.0;
        filteredDerivative = 0.0;
        lastTime = -1;
        lastTarget = 0.0;
        output = 0.0;
        appliedVoltageScale = 1.0;
        // target is intentionally left unchanged, matching sibling controllers.
    }

    @Override
    public void setTarget(Double target) {
        this.target = Objects.requireNonNull(target, "target must not be null");
    }

    @Override
    public Double getTarget() {
        return target;
    }

    @Override
    public Double getOutput() {
        return output;
    }

    @Override
    public boolean atTarget() {
        if (!hasUpdate) {
            return false;
        }
        return atTarget(lastError);
    }

    public boolean atTarget(double error) {
        return Math.abs(error) <= constants.tolerance;
    }
}
