package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.autotune.RelayTuner;
import org.firstinspires.ftc.teamcode.util.autotune.SvaFit;
import org.firstinspires.ftc.teamcode.util.autotune.VelocityFilter;
import org.firstinspires.ftc.teamcode.util.autotune.ZieglerNichols;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityConstants;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityController;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;

/**
 * Pedro Pathing AutoTune Procedure that tunes PIDSVA values for a
 * single-motor flywheel.
 *
 * <p>DRAFT ONLY — not yet applied to any repository. Completes the
 * stub at HEAD 31b4a45 (which collected inputs but discarded its
 * constructor arguments, constructed all-zero constants, and never
 * emitted results or code). Registration is a separate draft snippet
 * (see the draft README): one {@code @Tuner} factory in
 * {@code pedro/Tuning.java}. This file must not be copied into a repo
 * until the user approves the draft and supplies the
 * TODO-USER-CONSTANTS values.
 *
 * <p>Phases, in order: (1) breakaway probe for kS, (2) relay test for
 * PID (Ku/Tu via corrected bookkeeping, never Aarav's
 * {@code computeResult()} amplitude), (3) quasistatic stepped holds
 * for kS/kV, (4) dynamic trapezoid ramps for kA (profile acceleration
 * is the regressor), (5) least-squares SVA fit in the Procedure, then
 * (6) live validation of Ziegler-Nichols candidates in order
 * 2 (no overshoot) → 3 → 4, stopping at the first candidate that
 * passes all gates. If a phase fails, times out, or is stopped
 * without usable data, no results are emitted and the Procedure
 * aborts visibly — never with dummy or zero "tuned" values (the
 * {@code MotorPIDVelocityTuner} lesson).
 *
 * <p>Units: Procedure inputs are in RPM where labelled; all internal
 * math and controller state are in ticks/sec, seconds, and power
 * units, except that the SVA fit stores kS/kV/kA in volts when
 * voltage compensation is on. Conversion happens once, at the
 * Procedure boundary: {@code ticks/sec = RPM * ticksPerRev / 60}.
 * The motor is supplied as {@code Function<HardwareMap, DcMotorEx>}
 * (as in the stub) so the same Procedure adapts to different
 * flywheels; no "shooter" hardware name is hard-coded in the tuning
 * math. The {@code reversed} input follows the {@code TuningConfig}
 * convention: true when positive power produces a decreasing encoder
 * reading. Phases never set motor direction; the supplier owns
 * hardware configuration, and phases convert power/velocity signs
 * through the {@code reversed} flag.
 *
 * <p>Safety: every phase sets {@code RUN_WITHOUT_ENCODER} before
 * driving power, polls {@code isStopRequested()} every iteration,
 * enforces an elapsed-time timeout and an overspeed abort, and stops
 * the motor in a {@code finally} block on every exit path. No balls
 * may be loaded; the Procedure requires a confirmation screen before
 * spinning hardware and re-states the requirement in each phase
 * description. Live data during a phase is visible only via Driver
 * Station / FTC Dashboard telemetry — the AutoTune UI shows no live
 * values (Checkpoint finding C2).
 *
 * <p>Voltage compensation (default ON, decided 2026-11-01): the flag
 * is a Procedure input defaulting to {@code true} and a constants
 * field defaulting to {@code true}. When on, the SVA fit converts
 * each sample with its concurrent battery-voltage reading (kS/kV/kA
 * stored in volts) and the controller divides the feedforward voltage
 * by the fresh measured battery voltage each loop
 * ({@code u_ff = V_ff / Vbatt}); feedback is not divided. When the
 * reading is unavailable or implausible, the controller divides by
 * the nominal voltage for that cycle (see the controller Javadoc for
 * the exact validity range and fallback). When off, the fit is in
 * power units and the constants are valid only near the recorded
 * battery voltage.
 */
public class ShooterPIDSVATuner extends Procedure {

    // TODO-USER-CONSTANTS: confirm every default below against the actual
    // motor, shot table, and safe relay amplitude before first use.
    static final double DEFAULT_TICKS_PER_REV_FALLBACK = 28.0;
    static final double DEFAULT_TARGET_RPM = 2875.0;
    static final double DEFAULT_RELAY_AMPLITUDE = 0.9;
    static final double DEFAULT_HYSTERESIS_RPM = 30.0;
    static final double DEFAULT_RELAY_TIMEOUT_SECONDS = 15.0;
    static final double DEFAULT_FEEDFORWARD_SETTLE_SECONDS = 1.5;
    static final double DEFAULT_NOMINAL_VOLTAGE = 12.0;
    static final double DEFAULT_TOLERANCE_TICKS_PER_SEC = 30.0;
    static final double DEFAULT_INTEGRAL_LIMIT_TICKS = 1000.0;
    static final double DERIVATIVE_FILTER_TAU_SECONDS = 0.04;
    static final double VELOCITY_FILTER_TAU_SECONDS = 0.04;
    static final double OVERSPEED_MARGIN_TICKS_PER_SEC = 300.0;

    private final Function<HardwareMap, DcMotorEx> shooterMotorSupplier;
    private final double ticksPerRev;

    public ShooterPIDSVATuner(Function<HardwareMap, DcMotorEx> shooterMotorSupplier, double ticksPerRev) {
        super("Shooter PIDSVA Tuner", "Tunes PIDSVA values for the shooter");
        if (shooterMotorSupplier == null) {
            throw new IllegalArgumentException("shooterMotorSupplier must not be null.");
        }
        if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0) {
            throw new IllegalArgumentException("ticksPerRev must be finite and > 0.");
        }
        // FIX (stub defect): the stub discarded both constructor arguments.
        this.shooterMotorSupplier = shooterMotorSupplier;
        this.ticksPerRev = ticksPerRev;
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Tuning Inputs", "Inputs for the autotuner");
        Inputs.NumberField<Double> relayAmplitude = inputs.d("Relay Amplitude (bang-bang output magnitude)")
                .min(0.0).max(1.0).withDefault(DEFAULT_RELAY_AMPLITUDE);
        Inputs.NumberField<Integer> cyclesToCollect = inputs.i("Number of Oscillations (Cycles) to Collect")
                .min(4).withDefault(6);
        Inputs.NumberField<Integer> cyclesToIgnore = inputs.i("Number of Oscillations (Cycles) to Ignore")
                .min(0).withDefault(2);
        Inputs.NumberField<Double> relayTestTimeoutSecs = inputs.d("Relay Test Timeout (seconds)")
                .min(1.0).withDefault(DEFAULT_RELAY_TIMEOUT_SECONDS);
        Inputs.NumberField<Integer> targetRpm = inputs.i("Target RPM")
                .min(1).withDefault((int) DEFAULT_TARGET_RPM);
        Inputs.NumberField<Double> velocityHysteresisRpm = inputs.d("Velocity Hysteresis (Deadband) (RPM)")
                .min(0.0).withDefault(DEFAULT_HYSTERESIS_RPM);
        Inputs.StringField feedforwardTestPowers = inputs.s("Feedforward Test Powers (comma-separated)")
                .withDefault("0.15,0.25,0.35,0.50,0.75,1.00");
        Inputs.NumberField<Double> feedforwardSettleTimeSecs = inputs.d("Feedforward Settle Time (seconds)")
                .min(0.0).withDefault(DEFAULT_FEEDFORWARD_SETTLE_SECONDS);
        Inputs.Field<Boolean> tuneIntegralTerm = inputs.b("Tune Integral Term").withDefault(false);
        Inputs.Field<Boolean> voltageCompensation = inputs.b("Voltage compensation").withDefault(true);
        Inputs.NumberField<Double> nominalVoltage = inputs.d("Nominal Voltage (volts)")
                .min(6.0).max(15.0).withDefault(DEFAULT_NOMINAL_VOLTAGE);
        Inputs.Field<Boolean> reversed = inputs.b("Reversed (positive power decreases encoder reading)").withDefault(false);

        awaitInputs(inputs);

        // Defensive re-validation: Inputs validation failure is fatal to
        // the whole session with no re-prompt (Checkpoint finding C6),
        // so re-check here and abort visibly rather than starting a phase
        // with a bad value.
        double relayAmp = relayAmplitude.get();
        int collect = cyclesToCollect.get();
        int ignore = cyclesToIgnore.get();
        double relayTimeout = relayTestTimeoutSecs.get();
        int targetRpmValue = targetRpm.get();
        double hysteresisRpm = velocityHysteresisRpm.get();
        double settleSecs = feedforwardSettleTimeSecs.get();
        boolean tuneIntegral = tuneIntegralTerm.get();
        boolean voltageOn = voltageCompensation.get();
        double nominalVolts = nominalVoltage.get();
        boolean isReversed = reversed.get();
        List<Double> ffPowers;
        try {
            ffPowers = parseFeedforwardPowers(feedforwardTestPowers.get());
        } catch (IllegalArgumentException e) {
            abort("Invalid feedforward test powers: " + e.getMessage());
            return;
        }
        if (!(relayAmp > 0.0 && relayAmp <= 1.0) || collect < 4 || ignore < 0
                || relayTimeout <= 0.0 || targetRpmValue <= 0 || hysteresisRpm <= 0.0
                || settleSecs <= 0.0 || ffPowers.isEmpty() || nominalVolts <= 0.0) {
            abort("Invalid tuning inputs; check relay amplitude, cycles, target RPM, hysteresis, settle time, and nominal voltage.");
            return;
        }
        // Relay default choice (documented): 0.9 is kept as the default
        // because it is the TuningConfig placeholder the user asked to
        // model on, and it is overridable from this input. It is
        // aggressive for a first run (typical flywheel range 0.5-0.7);
        // above 0.7 the confirmation screen below says so explicitly,
        // so the operator must acknowledge it before any phase spins.
        String safetyWarning = relayAmp > 0.7
                ? " WARNING: relay amplitude " + relayAmp + " is aggressive (typical flywheel range 0.5-0.7). Consider stopping and lowering it for a first run."
                : "";

        confirmation("Flywheel tuning",
                "Are you ready to start tuning the flywheel? "
                        + "No balls may be loaded. Keep hands clear of the flywheel and confirm the area is clear. "
                        + "The wheel will spin at up to the target RPM and reverse briefly during the relay test."
                        + safetyWarning);

        double targetTicksPerSec = rpmToTicksPerSec(targetRpmValue);
        double hysteresisTicksPerSec = rpmToTicksPerSec(hysteresisRpm);

        // Phase 1: breakaway probe for kS.
        Double breakawayPower = runOpMode(new BreakawayPhase(
                shooterMotorSupplier, ticksPerRev, isReversed, voltageOn, nominalVolts));
        if (breakawayPower == null) {
            abort("Breakaway phase ended without a result. The wheel did not break away or the phase was stopped. Try again.");
            return;
        }

        // Phase 2: relay test for PID.
        RelayTuner.Result relayResult = runOpMode(new RelayPhase(
                shooterMotorSupplier, ticksPerRev, isReversed, voltageOn, nominalVolts,
                targetTicksPerSec, relayAmp, hysteresisTicksPerSec, collect, ignore, relayTimeout));
        if (relayResult == null || !relayResult.success) {
            String detail = relayResult == null ? "no result returned"
                    : relayResult.failure + ": " + relayResult.failureDetail;
            abort("Relay phase failed (" + detail + "). No Ku/Tu issued. Adjust relay amplitude, hysteresis, or target RPM and try again.");
            return;
        }

        // Phase 3: quasistatic stepped holds for kS/kV.
        List<SvaFit.Sample> quasistaticSamples = runOpMode(new QuasistaticPhase(
                shooterMotorSupplier, ticksPerRev, isReversed, voltageOn, nominalVolts, ffPowers, settleSecs));
        if (quasistaticSamples == null || quasistaticSamples.isEmpty()) {
            abort("Quasistatic phase ended without usable samples. Try again.");
            return;
        }

        // Preliminary kS/kV (breakaway + quasistatic line, intercept fixed
        // at kS) so the dynamic phase can follow a velocity profile.
        double[] preliminary = preliminaryKsKv(quasistaticSamples, breakawayPower, voltageOn, nominalVolts);
        double preliminaryKs = preliminary[0];
        double preliminaryKv = preliminary[1];

        // Phase 4: dynamic trapezoid ramps for kA, following a velocity
        // profile with the preliminary controller.
        List<SvaFit.Sample> dynamicSamples = runOpMode(new DynamicPhase(
                shooterMotorSupplier, ticksPerRev, isReversed, voltageOn, nominalVolts,
                preliminaryKs, preliminaryKv, relayResult, tuneIntegral));
        if (dynamicSamples == null || dynamicSamples.isEmpty()) {
            abort("Dynamic phase ended without usable samples. Try again.");
            return;
        }

        // Fit in the Procedure (pure math, no hardware).
        List<SvaFit.Sample> allSamples = new ArrayList<>(quasistaticSamples);
        allSamples.addAll(dynamicSamples);
        SvaFit.FitResult fit = SvaFit.fit(allSamples, voltageOn);
        if (!fit.accepted) {
            abort("SVA fit rejected: " + fit.rejectReason + " Re-run the missing phase; no constants are reported.");
            return;
        }
        if (fit.kARejected) {
            abort("SVA fit rejected the dynamic phase (kA materially negative). kA is not reported; re-run the dynamic phase.");
            return;
        }

        // Holding-power cross-check against the quasistatic level nearest the target.
        String holdCheck = holdingPowerCheck(quasistaticSamples, fit, targetTicksPerSec, voltageOn);

        // Candidates and validation, in order 2 -> 3 -> 4.
        List<ZieglerNichols.Candidate> candidates =
                ZieglerNichols.computeCandidates(relayResult.ku, relayResult.tuSeconds, tuneIntegral);
        int[] tryOrder = new int[]{2, 3, 4};
        List<ValidationResult> validationResults = new ArrayList<>();
        ZieglerNichols.Candidate chosenCandidate = null;
        DoubleFlywheelFullVelocityConstants chosenConstants = null;
        ValidationResult chosenValidation = null;
        for (int index : tryOrder) {
            ZieglerNichols.Candidate candidate = candidates.get(index);
            DoubleFlywheelFullVelocityConstants constants = buildConstants(
                    candidate, fit, targetTicksPerSec, tuneIntegral, voltageOn, nominalVolts);
            ValidationResult validation = runOpMode(new ValidationPhase(
                    shooterMotorSupplier, ticksPerRev, isReversed, voltageOn, nominalVolts,
                    constants, targetTicksPerSec, candidate.label));
            if (validation == null) {
                abort("Validation phase ended without a result for candidate " + candidate.label + ". Try again.");
                return;
            }
            validationResults.add(validation);
            if (chosenCandidate == null) {
                chosenCandidate = candidate;
                chosenConstants = constants;
                chosenValidation = validation;
            }
            if (validation.passed) {
                chosenCandidate = candidate;
                chosenConstants = constants;
                chosenValidation = validation;
                break;
            }
        }
        if (chosenCandidate == null || chosenConstants == null || chosenValidation == null) {
            abort("Validation produced no candidate. No constants are reported.");
            return;
        }
        boolean validationPassed = chosenValidation.passed;

        // Results: always at least two entries (Checkpoint finding C5).
        // Every value reported here was measured or fitted; nothing is a
        // placeholder or dummy (the MotorPIDVelocityTuner lesson).
        result("kP", format(chosenConstants.kP));
        result("kI", format(chosenConstants.kI));
        result("kD", format(chosenConstants.kD));
        result("kS", format(chosenConstants.kS));
        result("kV", format(chosenConstants.kV));
        result("kA", format(chosenConstants.kA));
        result("tolerance (ticks/sec)", format(chosenConstants.tolerance));
        result("integralLimit (ticks)", format(chosenConstants.integralLimit));
        result("voltageCompensation", Boolean.toString(voltageOn));
        result("nominalVoltage (V)", format(nominalVolts));
        result("fitUnits", fit.fittedInVolts ? "volts" : "power");
        result("Ku", format(relayResult.ku));
        result("Tu (s)", format(relayResult.tuSeconds));
        result("relayOscillationAmplitude", format(relayResult.amplitudeTicksPerSec));
        result("relayPeriodCv", format(relayResult.periodCv));
        result("relayAmplitudeCv", format(relayResult.amplitudeCv));
        result("relaySwitches", Integer.toString(relayResult.switchCount));
        result("relayDiscardedCycles", Integer.toString(relayResult.discardedCycleCount));
        result("relayWarnings", relayResult.warnings.isEmpty() ? "none" : String.join("; ", relayResult.warnings));
        result("breakawayPower", format(breakawayPower));
        result("preliminaryKs", format(preliminaryKs));
        result("preliminaryKv", format(preliminaryKv));
        result("fitRmse", format(fit.rmse));
        result("fitRSquared", format(fit.rSquared));
        result("fitScaledCondition", format(fit.scaledConditionNumber));
        result("fitSamplesUsed", Integer.toString(fit.samplesUsed));
        result("fitSamplesExcluded", Integer.toString(fit.samplesExcluded));
        result("fitWarnings", fit.warnings.isEmpty() ? "none" : String.join("; ", fit.warnings));
        result("holdCheck", holdCheck);
        result("targetRpm", Integer.toString(targetRpmValue));
        result("ticksPerRev", format(ticksPerRev));
        result("chosenCandidate", chosenCandidate.label);
        result("validationPassed", Boolean.toString(validationPassed));
        result("validationSpinUpSeconds", format(chosenValidation.spinUpSeconds));
        result("validationOvershootFraction", format(chosenValidation.overshootFraction));
        result("validationSettleSeconds", format(chosenValidation.settleSeconds));
        result("validationSteadyMeanError", format(chosenValidation.steadyMeanErrorTicksPerSec));
        result("validationSteadyStd", format(chosenValidation.steadyStdTicksPerSec));
        result("validationGates", chosenValidation.gateDetail.isEmpty() ? "all passed" : String.join("; ", chosenValidation.gateDetail));

        code(Language.JAVA, generatedConstantsCode(chosenConstants, relayResult, fit, targetRpmValue, voltageOn));
    }

    // ------------------------------------------------------------------
    // Helpers (Procedure boundary: RPM <-> ticks/sec, parsing, reporting)
    // ------------------------------------------------------------------

    double rpmToTicksPerSec(double rpm) {
        return rpm * ticksPerRev / 60.0;
    }

    static double ticksPerSecToRpm(double ticksPerSec, double ticksPerRev) {
        return ticksPerSec * 60.0 / ticksPerRev;
    }

    static List<Double> parseFeedforwardPowers(String raw) {
        if (raw == null || raw.trim().isEmpty()) {
            throw new IllegalArgumentException("feedforward test powers must not be empty.");
        }
        List<Double> powers = new ArrayList<>();
        for (String part : raw.split(",")) {
            String trimmed = part.trim();
            if (trimmed.isEmpty()) {
                continue;
            }
            double power;
            try {
                power = Double.parseDouble(trimmed);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("'" + trimmed + "' is not a number.", e);
            }
            if (!Double.isFinite(power) || power <= 0.0 || power > 1.0) {
                throw new IllegalArgumentException("power " + power + " must be in (0, 1].");
            }
            powers.add(power);
        }
        if (powers.isEmpty()) {
            throw new IllegalArgumentException("no usable powers parsed.");
        }
        return powers;
    }

    /** Preliminary kS (from breakaway) and kV (quasistatic line, intercept fixed at kS), in fitted units. */
    static double[] preliminaryKsKv(List<SvaFit.Sample> quasistaticSamples, double breakawayPower,
                                    boolean voltageOn, double nominalVolts) {
        double kS = voltageOn ? breakawayPower * nominalVolts : breakawayPower;
        double sumVv = 0.0;
        double sumYv = 0.0;
        for (SvaFit.Sample s : quasistaticSamples) {
            double y = voltageOn ? s.power * s.batteryVoltage : s.power;
            sumVv += s.velocityTicksPerSec * s.velocityTicksPerSec;
            sumYv += (y - kS) * s.velocityTicksPerSec;
        }
        double kV = sumVv > 0.0 ? sumYv / sumVv : 0.0;
        return new double[]{kS, kV};
    }

    /** Holding-power cross-check: fitted holding power at the target vs the measured quasistatic level nearest the target. */
    static String holdingPowerCheck(List<SvaFit.Sample> quasistaticSamples, SvaFit.FitResult fit,
                                    double targetTicksPerSec, boolean voltageOn) {
        SvaFit.Sample nearest = null;
        double nearestDiff = Double.POSITIVE_INFINITY;
        for (SvaFit.Sample s : quasistaticSamples) {
            double diff = Math.abs(s.velocityTicksPerSec - targetTicksPerSec);
            if (diff < nearestDiff) {
                nearestDiff = diff;
                nearest = s;
            }
        }
        if (nearest == null) {
            return "no quasistatic level recorded";
        }
        double fittedHolding = fit.kS + fit.kV * targetTicksPerSec;
        double measuredHolding = voltageOn ? nearest.power * nearest.batteryVoltage : nearest.power;
        double diff = Math.abs(fittedHolding - measuredHolding);
        double limit = voltageOn ? 1.2 : 0.10;
        String verdict = diff < limit ? "OK" : "HOLD_MISMATCH warning: difference exceeds " + limit + (voltageOn ? " V" : " power");
        return verdict + " (fitted " + format(fittedHolding) + ", measured " + format(measuredHolding) + ")";
    }

    static DoubleFlywheelFullVelocityConstants buildConstants(ZieglerNichols.Candidate candidate,
                                                              SvaFit.FitResult fit, double targetTicksPerSec,
                                                              boolean tuneIntegral, boolean voltageOn,
                                                              double nominalVolts) {
        double kI = tuneIntegral ? candidate.kI : 0.0;
        double integralLimit = kI != 0.0 ? 0.25 / Math.abs(kI) : DEFAULT_INTEGRAL_LIMIT_TICKS;
        double tolerance = Math.max(DEFAULT_TOLERANCE_TICKS_PER_SEC, 0.02 * targetTicksPerSec);
        return new DoubleFlywheelFullVelocityConstants(
                candidate.kP, kI, candidate.kD,
                fit.kS, fit.kV, fit.kA,
                tolerance, integralLimit,
                voltageOn, nominalVolts);
    }

    static String format(double value) {
        if (!Double.isFinite(value)) {
            return "NaN";
        }
        return String.format(Locale.US, "%.6g", value);
    }

    static String generatedConstantsCode(DoubleFlywheelFullVelocityConstants constants,
                                         RelayTuner.Result relayResult, SvaFit.FitResult fit,
                                         int targetRpm, boolean voltageOn) {
        StringBuilder sb = new StringBuilder();
        sb.append("// Generated by the Shooter PIDSVA Tuner (Pedro AutoTune Procedure).\n");
        sb.append("// Target: ").append(targetRpm).append(" RPM. Ku=").append(format(relayResult.ku));
        sb.append(", Tu=").append(format(relayResult.tuSeconds)).append(" s.\n");
        sb.append("// SVA fit: RMSE=").append(format(fit.rmse));
        sb.append(fit.fittedInVolts ? " V" : " power");
        sb.append(", R^2=").append(format(fit.rSquared));
        sb.append(", samples=").append(fit.samplesUsed).append(".\n");
        sb.append("// kS/kV/kA are in ").append(fit.fittedInVolts ? "volts" : "power units");
        sb.append(" (voltage compensation ").append(voltageOn ? "ON" : "OFF").append(").\n");
        sb.append("// TODO-USER-CONSTANTS: verify ticksPerRev and the shot table before use.\n");
        sb.append("DoubleFlywheelFullVelocityConstants constants = new DoubleFlywheelFullVelocityConstants(\n");
        sb.append("        ").append(format(constants.kP)).append(",  // kP\n");
        sb.append("        ").append(format(constants.kI)).append(",  // kI\n");
        sb.append("        ").append(format(constants.kD)).append(",  // kD\n");
        sb.append("        ").append(format(constants.kS)).append(",  // kS\n");
        sb.append("        ").append(format(constants.kV)).append(",  // kV\n");
        sb.append("        ").append(format(constants.kA)).append(",  // kA\n");
        sb.append("        ").append(format(constants.tolerance)).append(",  // tolerance (ticks/sec)\n");
        sb.append("        ").append(format(constants.integralLimit)).append(",  // integralLimit (ticks)\n");
        sb.append("        ").append(constants.voltageCompensation).append(",  // voltageCompensation\n");
        sb.append("        ").append(format(constants.nominalVoltage)).append("); // nominalVoltage (V)\n");
        return sb.toString();
    }

    // ------------------------------------------------------------------
    // Shared phase infrastructure
    // ------------------------------------------------------------------

    /**
     * Shared phase base. Subclasses implement the test loop; this base
     * owns motor setup (RUN_WITHOUT_ENCODER, FLOAT), sign conversion
     * for {@code reversed}, voltage reads, and the stop-on-every-exit
     * contract. Phases return null when stopped without usable data.
     */
    abstract static class FlywheelPhase<Result> extends TuningOpMode<Result> {
        final Function<HardwareMap, DcMotorEx> supplier;
        final double ticksPerRev;
        final boolean reversed;
        final boolean voltageCompensation;
        final double nominalVoltage;

        FlywheelPhase(String name, String description,
                      Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                      boolean reversed, boolean voltageCompensation, double nominalVoltage) {
            super(name, description, true);
            this.supplier = supplier;
            this.ticksPerRev = ticksPerRev;
            this.reversed = reversed;
            this.voltageCompensation = voltageCompensation;
            this.nominalVoltage = nominalVoltage;
        }

        DcMotorEx openMotor() {
            DcMotorEx motor = supplier.apply(hardwareMap);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0.0);
            return motor;
        }

        /** Effective (forward-positive) power: sign-converted for {@code reversed}. */
        void applyPower(DcMotorEx motor, double effectivePower) {
            motor.setPower(reversed ? -effectivePower : effectivePower);
        }

        /** Effective (forward-positive) velocity in ticks/sec. */
        double readVelocity(DcMotorEx motor) {
            double raw = motor.getVelocity();
            return reversed ? -raw : raw;
        }

        /** Best available battery voltage, or NaN when no valid sensor reading exists. */
        double readBatteryVoltage() {
            double best = Double.NaN;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double v = sensor.getVoltage();
                if (Double.isFinite(v) && v > 0.0 && (Double.isNaN(best) || v < best)) {
                    best = v;
                }
            }
            return best;
        }

        static double elapsedSeconds(long startNano) {
            return (System.nanoTime() - startNano) / 1e9;
        }

        void report(String label, Object value) {
            telemetry.addData(label, value);
        }
    }

    /** Breakaway probe: slow ramp 0 -> 0.35 over 6 s; power at first sustained motion is the result. */
    static final class BreakawayPhase extends FlywheelPhase<Double> {
        static final double RAMP_SECONDS = 6.0;
        static final double PEAK_POWER = 0.35;
        static final double MOTION_THRESHOLD_TICKS_PER_SEC = 50.0;
        static final double MOTION_SUSTAIN_SECONDS = 0.2;
        static final double TIMEOUT_SECONDS = 10.0;

        BreakawayPhase(Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                       boolean reversed, boolean voltageCompensation, double nominalVoltage) {
            super("Breakaway Probe", "Slow ramp to find breakaway power (kS). No balls loaded.",
                    supplier, ticksPerRev, reversed, voltageCompensation, nominalVoltage);
        }

        @Override
        protected Double runTuningOpMode() throws InterruptedException {
            waitForStart();
            DcMotorEx motor = openMotor();
            try {
                VelocityFilter filter = new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS);
                long start = System.nanoTime();
                long previous = start;
                long motionStart = -1;
                while (!isStopRequested()) {
                    long now = System.nanoTime();
                    double dt = Math.max((now - previous) / 1e9, 1e-3);
                    previous = now;
                    double elapsed = elapsedSeconds(start);
                    if (elapsed > TIMEOUT_SECONDS) {
                        break;
                    }
                    double power = PEAK_POWER * Math.min(1.0, elapsed / RAMP_SECONDS);
                    applyPower(motor, power);
                    double velocity = filter.update(readVelocity(motor), dt);
                    report("power", power);
                    report("velocity (ticks/s)", velocity);
                    telemetry.update();
                    if (velocity > MOTION_THRESHOLD_TICKS_PER_SEC) {
                        if (motionStart < 0) {
                            motionStart = System.nanoTime();
                        } else if ((System.nanoTime() - motionStart) / 1e9 >= MOTION_SUSTAIN_SECONDS) {
                            return power;
                        }
                    } else {
                        motionStart = -1;
                    }
                    sleep(10);
                }
                return null;
            } finally {
                motor.setPower(0.0);
            }
        }
    }

    /** Relay test: bang-bang relay around the target, with corrected per-interval extrema bookkeeping. */
    static final class RelayPhase extends FlywheelPhase<RelayTuner.Result> {
        private final double targetTicksPerSec;
        private final double relayAmplitude;
        private final double hysteresisTicksPerSec;
        private final int cyclesToCollect;
        private final int cyclesToIgnore;
        private final double timeoutSeconds;

        RelayPhase(Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                   boolean reversed, boolean voltageCompensation, double nominalVoltage,
                   double targetTicksPerSec, double relayAmplitude, double hysteresisTicksPerSec,
                   int cyclesToCollect, int cyclesToIgnore, double timeoutSeconds) {
            super("Relay Test", "Bang-bang relay test for Ku/Tu. No balls loaded; the wheel oscillates around the target.",
                    supplier, ticksPerRev, reversed, voltageCompensation, nominalVoltage);
            this.targetTicksPerSec = targetTicksPerSec;
            this.relayAmplitude = relayAmplitude;
            this.hysteresisTicksPerSec = hysteresisTicksPerSec;
            this.cyclesToCollect = cyclesToCollect;
            this.cyclesToIgnore = cyclesToIgnore;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        protected RelayTuner.Result runTuningOpMode() throws InterruptedException {
            waitForStart();
            DcMotorEx motor = openMotor();
            try {
                RelayTuner tuner = new RelayTuner(targetTicksPerSec, relayAmplitude, hysteresisTicksPerSec,
                        cyclesToCollect, cyclesToIgnore, timeoutSeconds, 3.0);
                VelocityFilter filter = new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS);
                long start = System.nanoTime();
                long previous = start;
                while (!isStopRequested()) {
                    long now = System.nanoTime();
                    double dt = (now - previous) / 1e9;
                    previous = now;
                    double elapsed = (now - start) / 1e9;
                    double velocity = filter.update(readVelocity(motor), Math.max(dt, 1e-3));
                    double power = tuner.update(velocity, elapsed);
                    if (power > 1.0) {
                        power = 1.0;
                    } else if (power < 0.0) {
                        power = 0.0;
                    }
                    applyPower(motor, power);
                    report("target (ticks/s)", targetTicksPerSec);
                    report("velocity (ticks/s)", velocity);
                    report("power", power);
                    report("switches", tuner.getSwitchCount());
                    report("battery (V)", readBatteryVoltage());
                    telemetry.update();
                    if (tuner.isFinished()) {
                        break;
                    }
                    sleep(5);
                }
                return tuner.computeResult();
            } finally {
                motor.setPower(0.0);
            }
        }
    }

    /** Quasistatic stepped holds: one settled mean sample per power level (plus a descending re-check). */
    static final class QuasistaticPhase extends FlywheelPhase<List<SvaFit.Sample>> {
        private final List<Double> powers;
        private final double settleSeconds;

        QuasistaticPhase(Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                         boolean reversed, boolean voltageCompensation, double nominalVoltage,
                         List<Double> powers, double settleSeconds) {
            super("Quasistatic Ramp", "Stepped power holds for kS/kV. No balls loaded.",
                    supplier, ticksPerRev, reversed, voltageCompensation, nominalVoltage);
            this.powers = new ArrayList<>(powers);
            this.settleSeconds = settleSeconds;
        }

        @Override
        protected List<SvaFit.Sample> runTuningOpMode() throws InterruptedException {
            waitForStart();
            DcMotorEx motor = openMotor();
            try {
                List<SvaFit.Sample> samples = new ArrayList<>();
                // Ascending sweep, then a descending re-check of the upper levels.
                List<Double> levels = new ArrayList<>(powers);
                int[] descending = new int[]{powers.size() - 2, powers.size() - 3, powers.size() - 4};
                for (int idx : descending) {
                    if (idx >= 0 && idx < powers.size()) {
                        levels.add(powers.get(idx));
                    }
                }
                for (int i = 0; i < levels.size(); i++) {
                    double power = levels.get(i);
                    String phaseLabel = i < powers.size() ? "quasistatic" : "quasistatic-descending";
                    VelocityFilter filter = new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS);
                    List<Double> windowVelocities = new ArrayList<>();
                    List<Double> windowVoltages = new ArrayList<>();
                    long start = System.nanoTime();
                    long previous = start;
                    while (!isStopRequested()) {
                        long now = System.nanoTime();
                        double dt = Math.max((now - previous) / 1e9, 1e-3);
                        previous = now;
                        double elapsed = (now - start) / 1e9;
                        if (elapsed >= settleSeconds) {
                            break;
                        }
                        applyPower(motor, power);
                        double velocity = filter.update(readVelocity(motor), dt);
                        if (elapsed >= settleSeconds - 0.5) {
                            windowVelocities.add(velocity);
                            double vBatt = readBatteryVoltage();
                            if (Double.isFinite(vBatt) && vBatt > 0.0) {
                                windowVoltages.add(vBatt);
                            }
                        }
                        report("power", power);
                        report("velocity (ticks/s)", velocity);
                        report("battery (V)", readBatteryVoltage());
                        telemetry.update();
                        sleep(10);
                    }
                    if (isStopRequested()) {
                        break;
                    }
                    if (!windowVelocities.isEmpty()) {
                        double mean = 0.0;
                        for (double v : windowVelocities) {
                            mean += v;
                        }
                        mean /= windowVelocities.size();
                        double variance = 0.0;
                        for (double v : windowVelocities) {
                            variance += (v - mean) * (v - mean);
                        }
                        double std = Math.sqrt(variance / windowVelocities.size());
                        double meanVoltage = nominalVoltage;
                        if (!windowVoltages.isEmpty()) {
                            double sum = 0.0;
                            for (double v : windowVoltages) {
                                sum += v;
                            }
                            meanVoltage = sum / windowVoltages.size();
                        }
                        // Valid only if the wheel is actually moving and settled.
                        if (mean > 50.0 && std < 0.05 * mean) {
                            samples.add(new SvaFit.Sample(mean, 0.0, power, meanVoltage, phaseLabel));
                        } else {
                            report("skipped level (unsettled/stalled)", power);
                            telemetry.update();
                        }
                    }
                }
                return samples;
            } finally {
                motor.setPower(0.0);
            }
        }
    }

    /** Dynamic trapezoid ramps: follows a velocity profile with the preliminary controller; regressor is profile acceleration. */
    static final class DynamicPhase extends FlywheelPhase<List<SvaFit.Sample>> {
        private static final double FAST_ACCEL_TICKS_PER_SEC_SQ = 2000.0;
        private static final double FAST_CEILING_TICKS_PER_SEC = 1000.0;
        private static final double SLOW_ACCEL_TICKS_PER_SEC_SQ = 100.0;
        private static final double SLOW_CEILING_TICKS_PER_SEC = 1200.0;
        private static final int FAST_REPEATS = 3;
        private static final double OVERSPEED_LIMIT_TICKS_PER_SEC = 1600.0;

        private final double preliminaryKs;
        private final double preliminaryKv;
        private final RelayTuner.Result relayResult;
        private final boolean tuneIntegral;

        DynamicPhase(Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                     boolean reversed, boolean voltageCompensation, double nominalVoltage,
                     double preliminaryKs, double preliminaryKv,
                     RelayTuner.Result relayResult, boolean tuneIntegral) {
            super("Dynamic Ramps", "Trapezoid velocity ramps for kA (profile acceleration is the regressor). No balls loaded.",
                    supplier, ticksPerRev, reversed, voltageCompensation, nominalVoltage);
            this.preliminaryKs = preliminaryKs;
            this.preliminaryKv = preliminaryKv;
            this.relayResult = relayResult;
            this.tuneIntegral = tuneIntegral;
        }

        @Override
        protected List<SvaFit.Sample> runTuningOpMode() throws InterruptedException {
            waitForStart();
            DcMotorEx motor = openMotor();
            try {
                List<ZieglerNichols.Candidate> candidates =
                        ZieglerNichols.computeCandidates(relayResult.ku, relayResult.tuSeconds, tuneIntegral);
                ZieglerNichols.Candidate candidate = candidates.get(2);
                double kI = tuneIntegral ? candidate.kI : 0.0;
                double integralLimit = kI != 0.0 ? 0.25 / Math.abs(kI) : DEFAULT_INTEGRAL_LIMIT_TICKS;
                DoubleFlywheelFullVelocityConstants preliminary = new DoubleFlywheelFullVelocityConstants(
                        candidate.kP, kI, candidate.kD,
                        preliminaryKs, preliminaryKv, 0.0,
                        DEFAULT_TOLERANCE_TICKS_PER_SEC, integralLimit,
                        voltageCompensation, nominalVoltage);
                DoubleFlywheelFullVelocityController controller =
                        new DoubleFlywheelFullVelocityController(preliminary);
                controller.setVoltageSupplier(this::readBatteryVoltage);

                // Segment table: duration, start velocity, end velocity, label.
                List<double[]> segments = new ArrayList<>();
                List<String> labels = new ArrayList<>();
                double fastUp = FAST_CEILING_TICKS_PER_SEC / FAST_ACCEL_TICKS_PER_SEC_SQ;
                for (int r = 0; r < FAST_REPEATS; r++) {
                    addSegment(segments, labels, fastUp, 0.0, FAST_CEILING_TICKS_PER_SEC, "fast-ramp-up");
                    addSegment(segments, labels, 1.0, FAST_CEILING_TICKS_PER_SEC, FAST_CEILING_TICKS_PER_SEC, "fast-hold");
                    addSegment(segments, labels, fastUp, FAST_CEILING_TICKS_PER_SEC, 0.0, "fast-ramp-down");
                    addSegment(segments, labels, 0.5, 0.0, 0.0, "fast-rest");
                }
                double slowUp = SLOW_CEILING_TICKS_PER_SEC / SLOW_ACCEL_TICKS_PER_SEC_SQ;
                addSegment(segments, labels, slowUp, 0.0, SLOW_CEILING_TICKS_PER_SEC, "slow-ramp");
                addSegment(segments, labels, 1.0, SLOW_CEILING_TICKS_PER_SEC, SLOW_CEILING_TICKS_PER_SEC, "slow-hold");
                addSegment(segments, labels, fastUp * (SLOW_CEILING_TICKS_PER_SEC / FAST_CEILING_TICKS_PER_SEC),
                        SLOW_CEILING_TICKS_PER_SEC, 0.0, "slow-ramp-down");

                List<SvaFit.Sample> samples = new ArrayList<>();
                VelocityFilter filter = new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS);
                long start = System.nanoTime();
                long previous = start;
                int segmentIndex = 0;
                long segmentStart = start;
                while (!isStopRequested() && segmentIndex < segments.size()) {
                    long now = System.nanoTime();
                    double dt = Math.max((now - previous) / 1e9, 1e-3);
                    previous = now;
                    double[] segment = segments.get(segmentIndex);
                    double segmentElapsed = (now - segmentStart) / 1e9;
                    if (segmentElapsed >= segment[0]) {
                        segmentIndex++;
                        segmentStart = now;
                        continue;
                    }
                    double fraction = segment[0] > 0.0 ? segmentElapsed / segment[0] : 1.0;
                    double profileVelocity = segment[1] + (segment[2] - segment[1]) * fraction;
                    double profileAccel = segment[0] > 0.0 ? (segment[2] - segment[1]) / segment[0] : 0.0;
                    double velocity = filter.update(readVelocity(motor), dt);
                    controller.setTarget(profileVelocity);
                    controller.update(velocity, dt);
                    double power = controller.getOutput();
                    applyPower(motor, power);
                    if (velocity > OVERSPEED_LIMIT_TICKS_PER_SEC) {
                        report("overspeed abort", velocity);
                        telemetry.update();
                        break;
                    }
                    // Exclude the first 0.10 s after a segment (direction) change: the
                    // filtered velocity is still catching up and the acceleration
                    // estimate is not representative of the profile.
                    if (segmentElapsed >= 0.10) {
                        double vBatt = readBatteryVoltage();
                        if (!Double.isFinite(vBatt) || vBatt <= 0.0) {
                            vBatt = nominalVoltage;
                        }
                        samples.add(new SvaFit.Sample(velocity, profileAccel, power, vBatt, labels.get(segmentIndex)));
                    }
                    report("target (ticks/s)", profileVelocity);
                    report("velocity (ticks/s)", velocity);
                    report("power", power);
                    report("segment", labels.get(segmentIndex));
                    telemetry.update();
                    sleep(5);
                }
                return samples;
            } finally {
                motor.setPower(0.0);
            }
        }

        private static void addSegment(List<double[]> segments, List<String> labels,
                                       double duration, double startV, double endV, String label) {
            segments.add(new double[]{duration, startV, endV});
            labels.add(label);
        }
    }

    /** Outcome of one live validation run against the gate set in the math design. */
    static final class ValidationResult {
        final boolean passed;
        final double spinUpSeconds;
        final double overshootFraction;
        final double settleSeconds;
        final double steadyMeanErrorTicksPerSec;
        final double steadyStdTicksPerSec;
        final double steadyMaxExcursionTicksPerSec;
        final int zeroCrossings;
        final List<String> gateDetail;

        ValidationResult(boolean passed, double spinUpSeconds, double overshootFraction,
                         double settleSeconds, double steadyMeanErrorTicksPerSec,
                         double steadyStdTicksPerSec, double steadyMaxExcursionTicksPerSec,
                         int zeroCrossings, List<String> gateDetail) {
            this.passed = passed;
            this.spinUpSeconds = spinUpSeconds;
            this.overshootFraction = overshootFraction;
            this.settleSeconds = settleSeconds;
            this.steadyMeanErrorTicksPerSec = steadyMeanErrorTicksPerSec;
            this.steadyStdTicksPerSec = steadyStdTicksPerSec;
            this.steadyMaxExcursionTicksPerSec = steadyMaxExcursionTicksPerSec;
            this.zeroCrossings = zeroCrossings;
            this.gateDetail = gateDetail;
        }
    }

    /** Live validation: ramp to the target, hold 5 s, ramp down; checks spin-up, overshoot, settle, and steady-state gates. */
    static final class ValidationPhase extends FlywheelPhase<ValidationResult> {
        private static final double RAMP_ACCEL_TICKS_PER_SEC_SQ = 2000.0;
        private static final double HOLD_SECONDS = 5.0;
        private static final double STEADY_WINDOW_SECONDS = 3.0;

        private final DoubleFlywheelFullVelocityConstants constants;
        private final double targetTicksPerSec;
        private final String candidateLabel;

        ValidationPhase(Function<HardwareMap, DcMotorEx> supplier, double ticksPerRev,
                        boolean reversed, boolean voltageCompensation, double nominalVoltage,
                        DoubleFlywheelFullVelocityConstants constants, double targetTicksPerSec,
                        String candidateLabel) {
            super("Validation: " + candidateLabel, "Live test of candidate " + candidateLabel + " at the target RPM. No balls loaded.",
                    supplier, ticksPerRev, reversed, voltageCompensation, nominalVoltage);
            this.constants = constants;
            this.targetTicksPerSec = targetTicksPerSec;
            this.candidateLabel = candidateLabel;
        }

        @Override
        protected ValidationResult runTuningOpMode() throws InterruptedException {
            waitForStart();
            DcMotorEx motor = openMotor();
            try {
                DoubleFlywheelFullVelocityController controller =
                        new DoubleFlywheelFullVelocityController(constants);
                controller.setVoltageSupplier(this::readBatteryVoltage);
                VelocityFilter filter = new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS);
                double tolerance = Math.max(constants.tolerance, 30.0);
                double rampSeconds = targetTicksPerSec / RAMP_ACCEL_TICKS_PER_SEC_SQ;
                double totalSeconds = rampSeconds + HOLD_SECONDS + rampSeconds;

                List<Double> errors = new ArrayList<>();
                List<Double> times = new ArrayList<>();
                double maxVelocity = 0.0;
                double firstBandTime = Double.NaN;
                double lastOutOfBandTime = Double.NaN;
                long start = System.nanoTime();
                long previous = start;
                while (!isStopRequested()) {
                    long now = System.nanoTime();
                    double dt = Math.max((now - previous) / 1e9, 1e-3);
                    previous = now;
                    double elapsed = (now - start) / 1e9;
                    if (elapsed >= totalSeconds) {
                        break;
                    }
                    double profileVelocity;
                    if (elapsed < rampSeconds) {
                        profileVelocity = targetTicksPerSec * (elapsed / rampSeconds);
                    } else if (elapsed < rampSeconds + HOLD_SECONDS) {
                        profileVelocity = targetTicksPerSec;
                    } else {
                        double downElapsed = elapsed - rampSeconds - HOLD_SECONDS;
                        profileVelocity = targetTicksPerSec * Math.max(0.0, 1.0 - downElapsed / rampSeconds);
                    }
                    double velocity = filter.update(readVelocity(motor), dt);
                    controller.setTarget(profileVelocity);
                    controller.update(velocity, dt);
                    applyPower(motor, controller.getOutput());
                    if (velocity > maxVelocity) {
                        maxVelocity = velocity;
                    }
                    if (elapsed >= rampSeconds && elapsed < rampSeconds + HOLD_SECONDS) {
                        double error = targetTicksPerSec - velocity;
                        errors.add(error);
                        times.add(elapsed - rampSeconds);
                        if (Math.abs(error) <= tolerance) {
                            if (Double.isNaN(firstBandTime)) {
                                firstBandTime = elapsed - rampSeconds;
                            }
                        } else {
                            lastOutOfBandTime = elapsed - rampSeconds;
                        }
                    }
                    report("target (ticks/s)", profileVelocity);
                    report("velocity (ticks/s)", velocity);
                    report("power", controller.getOutput());
                    telemetry.update();
                    sleep(5);
                }
                double spinUp = Double.isNaN(firstBandTime) ? Double.NaN : firstBandTime;
                double overshoot = targetTicksPerSec > 0.0 ? Math.max(0.0, maxVelocity / targetTicksPerSec - 1.0) : Double.NaN;
                // Settle: 0.5 s of continuous in-band after the last out-of-band sample; NaN if never settled.
                double settle = Double.NaN;
                if (!Double.isNaN(lastOutOfBandTime)) {
                    settle = lastOutOfBandTime + 0.5;
                    if (settle > HOLD_SECONDS) {
                        settle = Double.NaN;
                    }
                } else if (!errors.isEmpty()) {
                    settle = 0.0;
                }
                // Steady-state window: last STEADY_WINDOW_SECONDS of the hold.
                List<Double> steadyErrors = new ArrayList<>();
                for (int i = 0; i < errors.size(); i++) {
                    if (times.get(i) >= HOLD_SECONDS - STEADY_WINDOW_SECONDS) {
                        steadyErrors.add(errors.get(i));
                    }
                }
                double meanError = Double.NaN;
                double std = Double.NaN;
                double maxExcursion = Double.NaN;
                if (!steadyErrors.isEmpty()) {
                    double sum = 0.0;
                    for (double e : steadyErrors) {
                        sum += e;
                    }
                    meanError = sum / steadyErrors.size();
                    double variance = 0.0;
                    for (double e : steadyErrors) {
                        variance += (e - meanError) * (e - meanError);
                    }
                    std = Math.sqrt(variance / steadyErrors.size());
                    maxExcursion = 0.0;
                    for (double e : steadyErrors) {
                        maxExcursion = Math.max(maxExcursion, Math.abs(e));
                    }
                }
                // Ringing: zero-crossings of the error in the 2 s after settle.
                int crossings = 0;
                double settleStart = Double.isNaN(settle) ? 0.0 : settle;
                Double previousError = null;
                for (int i = 0; i < errors.size(); i++) {
                    if (times.get(i) >= settleStart && times.get(i) <= settleStart + 2.0) {
                        double e = errors.get(i);
                        if (previousError != null && Math.signum(e) != Math.signum(previousError) && e != 0.0) {
                            crossings++;
                        }
                        previousError = e;
                    }
                }
                List<String> gateDetail = new ArrayList<>();
                boolean passed = true;
                if (!(spinUp <= 1.5)) {
                    passed = false;
                    gateDetail.add("spin-up " + format(spinUp) + " s exceeds 1.5 s");
                }
                if (!(overshoot <= 0.05)) {
                    passed = false;
                    gateDetail.add("overshoot " + format(overshoot) + " exceeds 5%");
                }
                if (!(settle <= 2.0)) {
                    passed = false;
                    gateDetail.add("settle " + format(settle) + " s exceeds 2.0 s (or never settled)");
                }
                if (!Double.isFinite(meanError) || Math.abs(meanError) > tolerance) {
                    passed = false;
                    gateDetail.add("steady mean error " + format(meanError) + " outside tolerance " + format(tolerance));
                }
                if (!(std <= 0.015 * targetTicksPerSec)) {
                    passed = false;
                    gateDetail.add("steady std " + format(std) + " exceeds 1.5% of target");
                }
                if (!(maxExcursion <= 0.03 * targetTicksPerSec)) {
                    passed = false;
                    gateDetail.add("max excursion " + format(maxExcursion) + " exceeds 3% of target");
                }
                if (crossings > 4) {
                    passed = false;
                    gateDetail.add("ringing: " + crossings + " zero-crossings in 2 s after settle (limit 4)");
                }
                return new ValidationResult(passed, spinUp, overshoot, settle, meanError, std,
                        maxExcursion, crossings, gateDetail);
            } finally {
                motor.setPower(0.0);
            }
        }
    }
}
