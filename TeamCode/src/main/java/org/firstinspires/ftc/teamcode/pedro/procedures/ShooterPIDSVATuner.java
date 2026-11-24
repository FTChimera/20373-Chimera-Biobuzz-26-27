package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.rmsftc.utils.Clock;
import static org.firstinspires.ftc.teamcode.pedro.procedures.ShooterPIDSVATuner.FlywheelPhase.elapsedSeconds;

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
 * PIDSVA tuner for one or two shooter flywheels. With a single motor it
 * behaves exactly like the original single-motor tuner; with two motors
 * (main + counter roller) it runs both at the same time through the same
 * algorithm and reports a set of constants for each.
 *
 * <p>Phases, in order: (0a) manual spin direction, (0b) power direction
 * check, (1) breakaway probe for a preliminary kS, (2) relay test for
 * Ku/Tu and the PID candidates, (3) quasistatic stepped holds for kS/kV,
 * (4) dynamic trapezoid ramps for kA, (5) a three-parameter least-squares
 * fit of kS/kV/kA per motor, and (6) live validation of the
 * Ziegler-Nichols candidates in order 2 (no overshoot) -&gt; 3 -&gt; 4,
 * the first passing candidate winning per motor.
 *
 * <p>Direction signs are measured, not configured: the two setup OpModes
 * at the front of the Procedure find them per motor (manual spin for the
 * desired direction, +0.3 spin for the power direction), so the phases
 * work regardless of encoder wiring or whether getVelocity() is
 * direction-adjusted. A wrong hand-spin direction is harmless: the fit
 * is invariant to flipping both the recorded power and velocity signs.
 *
 * <p>Dual-motor notes: both wheels spin together in every powered phase
 * (same power profile / velocity profile / target), each with its own
 * preliminary controller, relay tuner, samples, fit, and validation
 * candidates. The manual-spin setup tracks both wheels' peaks in one
 * OpMode (spin the main by hand, then the counter roller by hand, then
 * stop it); the power-direction check spins both at +0.3 at once.
 * Validation picks each motor's first passing candidate independently,
 * so the two motors can end up on different candidates.
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

    /** One flywheel to tune: a name for labels/results and the motor supplier. */
    public static final class MotorSpec {
        public final String name;
        public final Function<HardwareMap, DcMotorEx> supplier;

        public MotorSpec(String name, Function<HardwareMap, DcMotorEx> supplier) {
            if (name == null || supplier == null) {
                throw new IllegalArgumentException("MotorSpec name and supplier must not be null.");
            }
            this.name = name;
            this.supplier = supplier;
        }
    }

    /** A motor being tuned: its spec plus the per-run direction signs and the opened motor. */
    static final class MotorChannel {
        final String name;
        final Function<HardwareMap, DcMotorEx> supplier;
        int desiredVelocitySign;
        int powerVelocitySign;
        DcMotorEx motor;

        MotorChannel(MotorSpec spec) {
            this.name = spec.name;
            this.supplier = spec.supplier;
        }
    }

    private final List<MotorChannel> channels;
    private final double ticksPerRev;

    /** Single-motor tuner (keeps the original behaviour and result keys). */
    public ShooterPIDSVATuner(Function<HardwareMap, DcMotorEx> shooterMotorSupplier, double ticksPerRev) {
        this(List.of(new MotorSpec("", shooterMotorSupplier)), ticksPerRev);
    }

    /** Multi-motor tuner: tunes every listed motor at the same time, same algorithm. */
    public ShooterPIDSVATuner(List<MotorSpec> motorSpecs, double ticksPerRev) {
        super("Shooter PIDSVA Tuner", "Tunes PIDSVA values for the shooter");
        if (motorSpecs == null || motorSpecs.isEmpty()) {
            throw new IllegalArgumentException("At least one motor is required.");
        }
        if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0) {
            throw new IllegalArgumentException("ticksPerRev must be finite and > 0.");
        }
        this.channels = new ArrayList<>();
        for (MotorSpec spec : motorSpecs) {
            if (spec == null) {
                throw new IllegalArgumentException("MotorSpec must not be null.");
            }
            this.channels.add(new MotorChannel(spec));
        }
        this.ticksPerRev = ticksPerRev;
    }

    /** True when more than one motor is being tuned (result keys are then prefixed per motor). */
    boolean isDual() {
        return channels.size() > 1;
    }

    /** Result key: unprefixed for a single motor, "name.key" for a dual run. */
    String key(MotorChannel channel, String base) {
        return isDual() ? channel.name + "." + base : base;
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
        // Direction signs are measured, not configured: the two setup
        // OpModes at the front of the Procedure find them (manual spin
        // for the desired direction, +0.3 spin for the power
        // direction). There is no "Reversed" input -- a manual reversal
        // flag only invited sign errors, spinning the wheel opposite
        // to the direction the operator wants and corrupting the
        // dynamic-phase fit into "kS materially negative".
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

        String motorWord = isDual() ? "both flywheels" : "the flywheel";
        String spinWord = isDual()
                ? "First spin the main flywheel BY HAND in the direction you want it to shoot, then spin the counter roller BY HAND in its shoot direction, then STOP that OpMode (like the Pinpoint encoder direction test) -- the motors are unpowered for that step. "
                : "First you will spin the wheel BY HAND in the direction you want it to shoot, then STOP that OpMode (like the Pinpoint encoder direction test) -- the motor is unpowered for that step. ";
        confirmation("Flywheel tuning",
                "Are you ready to start tuning " + motorWord + "? "
                        + spinWord
                        + "After that the powered phases run: no balls may be loaded. Keep hands clear of the flywheel and confirm the area is clear. "
                        + "The wheel will spin at up to the target RPM and reverse briefly during the relay test."
                        + safetyWarning);

        double targetTicksPerSec = rpmToTicksPerSec(targetRpmValue);
        double hysteresisTicksPerSec = rpmToTicksPerSec(hysteresisRpm);

        // Setup OpMode 1 of 2: manual spin -- the operator spins each
        // wheel by hand in its desired shoot direction, then stops the
        // OpMode. Returns the velocity sign per motor.
        List<Integer> desiredSigns = runOpMode(new ManualSpinPhase(channels));
        if (desiredSigns == null) {
            abort("Manual spin ended without a direction for every motor. Spin each flywheel by hand in the direction you want it to shoot, then STOP the OpMode, and try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            channels.get(i).desiredVelocitySign = desiredSigns.get(i);
        }

        // Setup OpMode 2 of 2: brief +0.3 spin -- learns which velocity
        // sign a positive power produces per motor, so the phases know
        // which raw power sign drives each wheel in its desired direction.
        List<Integer> powerSigns = runOpMode(new PowerDirectionPhase(channels));
        if (powerSigns == null) {
            abort("Power direction check ended without a result for every motor. A wheel did not move at +0.3 power; check the motor and try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            channels.get(i).powerVelocitySign = powerSigns.get(i);
        }
        for (MotorChannel ch : channels) {
            result(key(ch, "desired velocity sign (manual spin)"), Integer.toString(ch.desiredVelocitySign));
            result(key(ch, "power velocity sign (+0.3 power)"), Integer.toString(ch.powerVelocitySign));
        }

        // Phase 1: breakaway probe for kS, per motor.
        List<Double> breakawayPowers = runOpMode(new BreakawayPhase(
                channels, ticksPerRev, voltageOn, nominalVolts));
        if (breakawayPowers == null) {
            abort("Breakaway phase ended without a result. A wheel did not break away or the phase was stopped. Try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            if (breakawayPowers.get(i) == null) {
                abort("Breakaway phase: motor '" + channels.get(i).name + "' did not break away. Try again.");
                return;
            }
        }

        // Phase 2: relay test for PID, per motor.
        List<RelayTuner.Result> relayResults = runOpMode(new RelayPhase(
                channels, ticksPerRev, voltageOn, nominalVolts,
                targetTicksPerSec, relayAmp, hysteresisTicksPerSec, collect, ignore, relayTimeout));
        if (relayResults == null) {
            abort("Relay phase ended without a result. No Ku/Tu issued. Adjust relay amplitude, hysteresis, or target RPM and try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            RelayTuner.Result r = relayResults.get(i);
            if (r == null || !r.success) {
                String detail = r == null ? "no result returned"
                        : r.failure + ": " + r.failureDetail;
                abort("Relay phase failed for motor '" + channels.get(i).name + "' (" + detail + "). No Ku/Tu issued. Adjust relay amplitude, hysteresis, or target RPM and try again.");
                return;
            }
        }

        // Phase 3: quasistatic stepped holds for kS/kV, per motor.
        List<List<SvaFit.Sample>> quasistaticSamples = runOpMode(new QuasistaticPhase(
                channels, ticksPerRev, voltageOn, nominalVolts, ffPowers, settleSecs));
        if (quasistaticSamples == null) {
            abort("Quasistatic phase ended without usable samples. Try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            if (quasistaticSamples.get(i) == null || quasistaticSamples.get(i).isEmpty()) {
                abort("Quasistatic phase ended without usable samples for motor '" + channels.get(i).name + "'. Try again.");
                return;
            }
        }

        // Preliminary kS/kV per motor (breakaway + quasistatic line,
        // intercept fixed at kS) so the dynamic phase can follow a
        // velocity profile.
        List<Double> preliminaryKs = new ArrayList<>();
        List<Double> preliminaryKv = new ArrayList<>();
        for (int i = 0; i < channels.size(); i++) {
            double[] preliminary = preliminaryKsKv(quasistaticSamples.get(i), breakawayPowers.get(i), voltageOn, nominalVolts);
            preliminaryKs.add(preliminary[0]);
            preliminaryKv.add(preliminary[1]);
        }

        // Phase 4: dynamic trapezoid ramps for kA, per motor, following
        // a velocity profile with the preliminary controller.
        List<List<SvaFit.Sample>> dynamicSamples = runOpMode(new DynamicPhase(
                channels, ticksPerRev, voltageOn, nominalVolts,
                preliminaryKs, preliminaryKv, relayResults, tuneIntegral));
        if (dynamicSamples == null) {
            abort("Dynamic phase ended without usable samples. Try again.");
            return;
        }
        for (int i = 0; i < channels.size(); i++) {
            if (dynamicSamples.get(i) == null || dynamicSamples.get(i).isEmpty()) {
                abort("Dynamic phase ended without usable samples for motor '" + channels.get(i).name + "'. Try again.");
                return;
            }
        }

        // Fit per motor (pure math, no hardware).
        List<SvaFit.FitResult> fits = new ArrayList<>();
        for (int i = 0; i < channels.size(); i++) {
            List<SvaFit.Sample> allSamples = new ArrayList<>(quasistaticSamples.get(i));
            allSamples.addAll(dynamicSamples.get(i));
            SvaFit.FitResult fit = SvaFit.fit(allSamples, voltageOn);
            if (!fit.accepted) {
                abort("SVA fit rejected for motor '" + channels.get(i).name + "': " + fit.rejectReason + " Re-run the missing phase; no constants are reported.");
                return;
            }
            if (fit.kARejected) {
                abort("SVA fit rejected the dynamic phase for motor '" + channels.get(i).name + "' (kA materially negative). kA is not reported; re-run the dynamic phase.");
                return;
            }
            fits.add(fit);
        }

        // Holding-power cross-check per motor, against the quasistatic
        // level nearest the target.
        List<String> holdChecks = new ArrayList<>();
        for (int i = 0; i < channels.size(); i++) {
            holdChecks.add(holdingPowerCheck(quasistaticSamples.get(i), fits.get(i), targetTicksPerSec, voltageOn));
        }

        // Candidates and validation, in order 2 -> 3 -> 4, per motor.
        // Each motor picks its first passing candidate independently.
        List<List<ZieglerNichols.Candidate>> candidatesPerMotor = new ArrayList<>();
        for (int i = 0; i < channels.size(); i++) {
            candidatesPerMotor.add(ZieglerNichols.computeCandidates(relayResults.get(i).ku, relayResults.get(i).tuSeconds, tuneIntegral));
        }
        int[] tryOrder = new int[]{2, 3, 4};
        List<ZieglerNichols.Candidate> chosenCandidates = new ArrayList<>();
        List<DoubleFlywheelFullVelocityConstants> chosenConstantsList = new ArrayList<>();
        List<ValidationResult> chosenValidations = new ArrayList<>();
        List<Boolean> passedFlags = new ArrayList<>();
        List<ValidationResult> firstValidations = new ArrayList<>();
        for (int i = 0; i < channels.size(); i++) {
            chosenCandidates.add(null);
            chosenConstantsList.add(null);
            chosenValidations.add(null);
            passedFlags.add(false);
            firstValidations.add(null);
        }
        for (int round = 0; round < tryOrder.length; round++) {
            int index = tryOrder[round];
            boolean anyUnpassed = false;
            for (int i = 0; i < channels.size(); i++) {
                if (!passedFlags.get(i)) {
                    anyUnpassed = true;
                    break;
                }
            }
            if (!anyUnpassed) {
                break;
            }
            // Constants for this round: the candidate at this index for
            // motors still unpassed, the already-chosen constants for
            // motors that passed (they spin along; their result is ignored).
            List<DoubleFlywheelFullVelocityConstants> roundConstants = new ArrayList<>();
            for (int i = 0; i < channels.size(); i++) {
                if (passedFlags.get(i)) {
                    roundConstants.add(chosenConstantsList.get(i));
                } else {
                    roundConstants.add(buildConstants(
                            candidatesPerMotor.get(i).get(index), fits.get(i), targetTicksPerSec,
                            tuneIntegral, voltageOn, nominalVolts));
                }
            }
            List<ValidationResult> roundResults = runOpMode(new ValidationPhase(
                    channels, ticksPerRev, voltageOn, nominalVolts,
                    roundConstants, targetTicksPerSec,
                    "candidate " + candidatesPerMotor.get(0).get(index).label));
            if (roundResults == null) {
                abort("Validation phase ended without a result for candidate index " + index + ". Try again.");
                return;
            }
            for (int i = 0; i < channels.size(); i++) {
                if (passedFlags.get(i)) {
                    continue;
                }
                ValidationResult validation = roundResults.get(i);
                if (validation == null) {
                    abort("Validation phase ended without a result for motor '" + channels.get(i).name + "' at candidate index " + index + ". Try again.");
                    return;
                }
                if (round == 0) {
                    firstValidations.set(i, validation);
                }
                if (validation.passed) {
                    chosenCandidates.set(i, candidatesPerMotor.get(i).get(index));
                    chosenConstantsList.set(i, roundConstants.get(i));
                    chosenValidations.set(i, validation);
                    passedFlags.set(i, true);
                }
            }
        }
        // Fallback for motors that never passed: the first tried candidate
        // (index 2) with its validation result, matching the single-motor tuner.
        for (int i = 0; i < channels.size(); i++) {
            if (!passedFlags.get(i)) {
                chosenCandidates.set(i, candidatesPerMotor.get(i).get(tryOrder[0]));
                chosenConstantsList.set(i, buildConstants(
                        candidatesPerMotor.get(i).get(tryOrder[0]), fits.get(i), targetTicksPerSec,
                        tuneIntegral, voltageOn, nominalVolts));
                chosenValidations.set(i, firstValidations.get(i));
            }
            if (chosenCandidates.get(i) == null || chosenConstantsList.get(i) == null || chosenValidations.get(i) == null) {
                abort("Validation produced no candidate for motor '" + channels.get(i).name + "'. No constants are reported.");
                return;
            }
        }

        // Results per motor: always at least two entries (Checkpoint finding C5).
        // Every value reported here was measured or fitted; nothing is a
        // placeholder or dummy (the MotorPIDVelocityTuner lesson).
        for (int i = 0; i < channels.size(); i++) {
            MotorChannel ch = channels.get(i);
            DoubleFlywheelFullVelocityConstants constants = chosenConstantsList.get(i);
            SvaFit.FitResult fit = fits.get(i);
            RelayTuner.Result relayResult = relayResults.get(i);
            ValidationResult validation = chosenValidations.get(i);
            result(key(ch, "kP"), format(constants.kP));
            result(key(ch, "kI"), format(constants.kI));
            result(key(ch, "kD"), format(constants.kD));
            result(key(ch, "kS"), format(constants.kS));
            result(key(ch, "kV"), format(constants.kV));
            result(key(ch, "kA"), format(constants.kA));
            result(key(ch, "tolerance (ticks/sec)"), format(constants.tolerance));
            result(key(ch, "integralLimit (ticks)"), format(constants.integralLimit));
            result(key(ch, "voltageCompensation"), Boolean.toString(voltageOn));
            result(key(ch, "nominalVoltage (V)"), format(nominalVolts));
            result(key(ch, "fitUnits"), fit.fittedInVolts ? "volts" : "power");
            result(key(ch, "Ku"), format(relayResult.ku));
            result(key(ch, "Tu (s)"), format(relayResult.tuSeconds));
            result(key(ch, "relayOscillationAmplitude"), format(relayResult.amplitudeTicksPerSec));
            result(key(ch, "relayPeriodCv"), format(relayResult.periodCv));
            result(key(ch, "relayAmplitudeCv"), format(relayResult.amplitudeCv));
            result(key(ch, "relaySwitches"), Integer.toString(relayResult.switchCount));
            result(key(ch, "relayDiscardedCycles"), Integer.toString(relayResult.discardedCycleCount));
            result(key(ch, "relayWarnings"), relayResult.warnings.isEmpty() ? "none" : String.join("; ", relayResult.warnings));
            result(key(ch, "breakawayPower"), format(breakawayPowers.get(i)));
            result(key(ch, "preliminaryKs"), format(preliminaryKs.get(i)));
            result(key(ch, "preliminaryKv"), format(preliminaryKv.get(i)));
            result(key(ch, "fitRmse"), format(fit.rmse));
            result(key(ch, "fitRSquared"), format(fit.rSquared));
            result(key(ch, "fitScaledCondition"), format(fit.scaledConditionNumber));
            result(key(ch, "fitSamplesUsed"), Integer.toString(fit.samplesUsed));
            result(key(ch, "fitSamplesExcluded"), Integer.toString(fit.samplesExcluded));
            result(key(ch, "fitWarnings"), fit.warnings.isEmpty() ? "none" : String.join("; ", fit.warnings));
            result(key(ch, "holdCheck"), holdChecks.get(i));
            result(key(ch, "targetRpm"), Integer.toString(targetRpmValue));
            result(key(ch, "ticksPerRev"), format(ticksPerRev));
            result(key(ch, "chosenCandidate"), chosenCandidates.get(i).label);
            result(key(ch, "validationPassed"), Boolean.toString(validation.passed));
            result(key(ch, "validationSpinUpSeconds"), format(validation.spinUpSeconds));
            result(key(ch, "validationOvershootFraction"), format(validation.overshootFraction));
            result(key(ch, "validationSettleSeconds"), format(validation.settleSeconds));
            result(key(ch, "validationSteadyMeanError"), format(validation.steadyMeanErrorTicksPerSec));
            result(key(ch, "validationSteadyStd"), format(validation.steadyStdTicksPerSec));
            result(key(ch, "validationGates"), validation.gateDetail.isEmpty() ? "all passed" : String.join("; ", validation.gateDetail));
        }

        code(Language.JAVA, generatedConstantsCode(chosenConstantsList, relayResults, fits, targetRpmValue, voltageOn));
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

    /** Generated constants for every tuned motor, one block per motor. */
    String generatedConstantsCode(List<DoubleFlywheelFullVelocityConstants> constantsList,
                                  List<RelayTuner.Result> relayResults, List<SvaFit.FitResult> fits,
                                  int targetRpm, boolean voltageOn) {
        StringBuilder sb = new StringBuilder();
        sb.append("// Generated by the Shooter PIDSVA Tuner (Pedro AutoTune Procedure).\n");
        sb.append("// Target: ").append(targetRpm).append(" RPM.\n");
        for (int i = 0; i < channels.size(); i++) {
            MotorChannel ch = channels.get(i);
            DoubleFlywheelFullVelocityConstants constants = constantsList.get(i);
            RelayTuner.Result relayResult = relayResults.get(i);
            SvaFit.FitResult fit = fits.get(i);
            String label = isDual() ? ch.name : "shooter";
            sb.append("// ---- ").append(label).append(" ----\n");
            sb.append("// Ku=").append(format(relayResult.ku));
            sb.append(", Tu=").append(format(relayResult.tuSeconds)).append(" s.\n");
            sb.append("// SVA fit: RMSE=").append(format(fit.rmse));
            sb.append(fit.fittedInVolts ? " V" : " power");
            sb.append(", R^2=").append(format(fit.rSquared));
            sb.append(", samples=").append(fit.samplesUsed).append(".\n");
            sb.append("// kS/kV/kA are in ").append(fit.fittedInVolts ? "volts" : "power units");
            sb.append(" (voltage compensation ").append(voltageOn ? "ON" : "OFF").append(").\n");
            sb.append("// TODO-USER-CONSTANTS: verify ticksPerRev and the shot table before use.\n");
            sb.append("DoubleFlywheelFullVelocityConstants ").append(isDual() ? ch.name + "Constants" : "constants");
            sb.append(" = new DoubleFlywheelFullVelocityConstants(\n");
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
        }
        return sb.toString();
    }

    // ------------------------------------------------------------------
    // Shared phase infrastructure
    // ------------------------------------------------------------------

    /**
     * Shared phase base. Subclasses implement the test loop; this base
     * owns motor setup (RUN_WITHOUT_ENCODER, FLOAT), sign conversion
     * for the two direction signs measured by the setup OpModes
     * (desiredVelocitySign, powerVelocitySign), voltage reads, and
     * the stop-on-every-exit contract. Phases return null when
     * stopped without usable data.
     */
    abstract static class FlywheelPhase<Result> extends TuningOpMode<Result> {
        final List<MotorChannel> channels;
        final double ticksPerRev;
        // Direction signs, measured once per tuner run by the two setup
        // OpModes at the front of the Procedure (ManualSpinPhase and
        // PowerDirectionPhase) and shared by every phase after them:
        // desiredVelocitySign is the velocity sign for the spin
        // direction the operator wants (found by spinning the wheel by
        // hand); powerVelocitySign is the velocity sign that a positive
        // raw power produces (found by a brief +0.3 spin). Positive
        // effective power always spins the wheel in the desired
        // direction, and a positive recorded velocity always means the
        // desired direction -- regardless of encoder wiring or whether
        // getVelocity() is direction-adjusted. That is what keeps the
        // SVA fit from seeing opposite-sign (power, velocity) data and
        // rejecting with "kS materially negative" / "kV is not
        // positive".
        final boolean voltageCompensation;
        final double nominalVoltage;

        FlywheelPhase(String name, String description,
                      List<MotorChannel> channels, double ticksPerRev,
                      boolean voltageCompensation, double nominalVoltage) {
            super(name, description, true);
            this.channels = channels;
            this.ticksPerRev = ticksPerRev;
            this.voltageCompensation = voltageCompensation;
            this.nominalVoltage = nominalVoltage;
        }

        /** Opens every channel's motor (RUN_WITHOUT_ENCODER, FLOAT, power 0). */
        List<DcMotorEx> openMotors() {
            List<DcMotorEx> motors = new ArrayList<>();
            for (MotorChannel ch : channels) {
                DcMotorEx motor = ch.supplier.apply(hardwareMap);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setPower(0.0);
                ch.motor = motor;
                motors.add(motor);
            }
            return motors;
        }

        /** Stops every channel's motor. */
        void stopMotors() {
            for (MotorChannel ch : channels) {
                if (ch.motor != null) {
                    ch.motor.setPower(0.0);
                }
            }
        }

        /**
         * Effective (forward-positive) power: positive always spins
         * the wheel in the desired direction. The raw power applied to
         * the motor is the effective power times
         * desiredVelocitySign * powerVelocitySign: since a raw power
         * of sign s gives a velocity of sign s * powerVelocitySign,
         * applying s = desiredVelocitySign * powerVelocitySign gives
         * a velocity of sign desiredVelocitySign, which is the
         * direction the operator wants.
         */
        void applyPower(MotorChannel channel, double effectivePower) {
            channel.motor.setPower(effectivePower * channel.desiredVelocitySign * channel.powerVelocitySign);
        }

        /**
         * Effective (forward-positive) velocity in ticks/sec: positive
         * always means spinning in the desired direction.
         */
        double readVelocity(MotorChannel channel) {
            return channel.motor.getVelocity() * channel.desiredVelocitySign;
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
            return (Clock.SYSTEM.nowNanos() - startNano) / 1e9;
        }

        void report(String label, Object value) {
            telemetry.addData(label, value);
        }
    }

    /**
     * Setup OpMode 1 of 2 (runs first, before any powered phase): the
     * operator spins each flywheel BY HAND in the direction they want
     * it to spin for shooting, then STOPS the OpMode -- the same
     * "you stop it" flow as the Pinpoint tuner forward/strafe encoder
     * direction tests. The motors are never powered here; this only
     * reads the velocity encoders. Returns the velocity sign (+1 or
     * -1) per motor, or null when a wheel was never spun fast enough
     * to tell. For a dual run, spin the main wheel, then the counter
     * roller, then stop the OpMode; both peaks are tracked at once.
     */
    static final class ManualSpinPhase extends TuningOpMode<List<Integer>> {
        static final double MOTION_THRESHOLD_TICKS_PER_SEC = 50.0;
        final List<MotorChannel> channels;

        ManualSpinPhase(List<MotorChannel> channels) {
            super("Manual Spin Direction", "Spin each flywheel BY HAND in the direction you want it to shoot, then STOP the OpMode.", true);
            this.channels = channels;
        }

        @Override
        protected List<Integer> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = new ArrayList<>();
            for (MotorChannel ch : channels) {
                DcMotorEx motor = ch.supplier.apply(hardwareMap);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setPower(0.0);
                ch.motor = motor;
                motors.add(motor);
            }
            try {
                List<Double> peakVelocities = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    peakVelocities.add(0.0);
                }
                while (!isStopRequested()) {
                    for (int i = 0; i < channels.size(); i++) {
                        double velocity = motors.get(i).getVelocity();
                        if (Math.abs(velocity) > Math.abs(peakVelocities.get(i))) {
                            peakVelocities.set(i, velocity);
                        }
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        telemetry.addData(label + " live velocity (ticks/sec)", "%.0f", velocity);
                        telemetry.addData(label + " peak velocity (ticks/sec)", "%.0f", peakVelocities.get(i));
                        telemetry.addData(label + " detected desired sign",
                                Math.abs(peakVelocities.get(i)) > MOTION_THRESHOLD_TICKS_PER_SEC
                                        ? (peakVelocities.get(i) > 0 ? "+1 (positive velocity)" : "-1 (negative velocity)")
                                        : "not yet -- spin it faster");
                    }
                    telemetry.addData("Spin each wheel by hand (desired shoot direction), then STOP", "");
                    telemetry.update();
                    sleep(50);
                }
                // Stopped by the operator: each peak velocity's sign is
                // that motor's desired direction.
                List<Integer> signs = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    double peak = peakVelocities.get(i);
                    if (Math.abs(peak) <= MOTION_THRESHOLD_TICKS_PER_SEC) {
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        telemetry.addData("manual spin", "no spin detected for " + label + " (peak " + peak + " ticks/sec); spin the wheel faster and try again");
                        return null;
                    }
                    signs.add(peak > 0 ? 1 : -1);
                }
                for (int i = 0; i < channels.size(); i++) {
                    String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                    telemetry.addData(label + " desired velocity sign (manual spin)", signs.get(i));
                    telemetry.addData(label + " peak velocity (ticks/sec)", peakVelocities.get(i));
                }
                return signs;
            } finally {
                for (DcMotorEx motor : motors) {
                    motor.setPower(0.0);
                }
            }
        }
    }

    /**
     * Setup OpMode 2 of 2 (runs after the manual spin, before any
     * powered phase): spins every wheel briefly at a raw +0.3 power and
     * measures which velocity sign a positive power produces per motor.
     * Returns that sign (+1 or -1) per motor, or null when a wheel
     * never moved enough to tell. Together with the manual spin sign,
     * this tells the phases which raw power sign drives each wheel in
     * the direction the operator wants.
     */
    static final class PowerDirectionPhase extends TuningOpMode<List<Integer>> {
        static final double TEST_POWER = 0.3;
        static final double MOTION_THRESHOLD_TICKS_PER_SEC = 50.0;
        static final double TIMEOUT_SECONDS = 2.0;
        final List<MotorChannel> channels;

        PowerDirectionPhase(List<MotorChannel> channels) {
            super("Power Direction Check", "Brief +0.3 spin to learn which velocity sign positive power gives. No balls loaded.", true);
            this.channels = channels;
        }

        @Override
        protected List<Integer> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = new ArrayList<>();
            for (MotorChannel ch : channels) {
                DcMotorEx motor = ch.supplier.apply(hardwareMap);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setPower(0.0);
                ch.motor = motor;
                motors.add(motor);
            }
            try {
                // Raw +0.3 on every motor at once, bypassing any sign
                // conversion: the raw power-to-velocity relationship is
                // what is measured.
                for (DcMotorEx motor : motors) {
                    motor.setPower(TEST_POWER);
                }
                long start = Clock.SYSTEM.nowNanos();
                List<Double> velocities = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    velocities.add(0.0);
                }
                while (!isStopRequested() && elapsedSeconds(start) < TIMEOUT_SECONDS) {
                    sleep(50);
                    boolean allMoved = true;
                    for (int i = 0; i < channels.size(); i++) {
                        double velocity = motors.get(i).getVelocity();
                        velocities.set(i, velocity);
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        telemetry.addData(label + " live velocity (ticks/sec)", "%.0f", velocity);
                        if (Math.abs(velocity) <= MOTION_THRESHOLD_TICKS_PER_SEC) {
                            allMoved = false;
                        }
                    }
                    telemetry.addData("Testing +0.3 power...", "");
                    telemetry.update();
                    if (allMoved) {
                        break;
                    }
                }
                for (DcMotorEx motor : motors) {
                    motor.setPower(0.0);
                }
                List<Integer> signs = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    double velocity = velocities.get(i);
                    if (Math.abs(velocity) <= MOTION_THRESHOLD_TICKS_PER_SEC) {
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        telemetry.addData("power direction check", label + " did not move (velocity " + velocity + " ticks/sec at +0.3); check the motor and try again");
                        return null;
                    }
                    signs.add(velocity > 0 ? 1 : -1);
                }
                for (int i = 0; i < channels.size(); i++) {
                    String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                    telemetry.addData(label + " power velocity sign (+0.3 power)", signs.get(i));
                    telemetry.addData(label + " velocity at +0.3 (ticks/sec)", velocities.get(i));
                }
                return signs;
            } finally {
                for (DcMotorEx motor : motors) {
                    motor.setPower(0.0);
                }
            }
        }
    }

    /** Breakaway probe: slow ramp 0 -> 0.35 over 6 s; power at first sustained motion is the result, per motor. */
    static final class BreakawayPhase extends FlywheelPhase<List<Double>> {
        static final double RAMP_SECONDS = 6.0;
        static final double PEAK_POWER = 0.35;
        static final double MOTION_THRESHOLD_TICKS_PER_SEC = 50.0;
        static final double MOTION_SUSTAIN_SECONDS = 0.2;
        static final double TIMEOUT_SECONDS = 10.0;

        BreakawayPhase(List<MotorChannel> channels, double ticksPerRev,
                       boolean voltageCompensation, double nominalVoltage) {
            super("Breakaway Probe", "Slow ramp to find breakaway power (kS). No balls loaded.",
                    channels, ticksPerRev, voltageCompensation, nominalVoltage);
        }

        @Override
        protected List<Double> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = openMotors();
            try {
                List<VelocityFilter> filters = new ArrayList<>();
                List<Double> breakawayPowers = new ArrayList<>();
                List<Long> motionStarts = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    filters.add(new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS));
                    breakawayPowers.add(null);
                    motionStarts.add(-1L);
                }
                long start = Clock.SYSTEM.nowNanos();
                long previous = start;
                while (!isStopRequested()) {
                    long now = Clock.SYSTEM.nowNanos();
                    double dt = Math.max((now - previous) / 1e9, 1e-3);
                    previous = now;
                    double elapsed = elapsedSeconds(start);
                    if (elapsed > TIMEOUT_SECONDS) {
                        break;
                    }
                    double power = PEAK_POWER * Math.min(1.0, elapsed / RAMP_SECONDS);
                    boolean allFound = true;
                    for (int i = 0; i < channels.size(); i++) {
                        if (breakawayPowers.get(i) != null) {
                            continue;
                        }
                        allFound = false;
                        applyPower(channels.get(i), power);
                        double velocity = filters.get(i).update(readVelocity(channels.get(i)), dt);
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        report(label + " power", power);
                        report(label + " velocity (ticks/s)", velocity);
                        if (velocity > MOTION_THRESHOLD_TICKS_PER_SEC) {
                            if (motionStarts.get(i) < 0) {
                                motionStarts.set(i, Clock.SYSTEM.nowNanos());
                            } else if ((Clock.SYSTEM.nowNanos() - motionStarts.get(i)) / 1e9 >= MOTION_SUSTAIN_SECONDS) {
                                breakawayPowers.set(i, power);
                            }
                        } else {
                            motionStarts.set(i, -1L);
                        }
                    }
                    telemetry.update();
                    if (allFound) {
                        break;
                    }
                    sleep(10);
                }
                return breakawayPowers;
            } finally {
                stopMotors();
            }
        }
    }

    /** Relay test: bang-bang relay around the target, with corrected per-interval extrema bookkeeping, per motor. */
    static final class RelayPhase extends FlywheelPhase<List<RelayTuner.Result>> {
        private final double targetTicksPerSec;
        private final double relayAmplitude;
        private final double hysteresisTicksPerSec;
        private final int cyclesToCollect;
        private final int cyclesToIgnore;
        private final double timeoutSeconds;

        RelayPhase(List<MotorChannel> channels, double ticksPerRev,
                   boolean voltageCompensation, double nominalVoltage,
                   double targetTicksPerSec, double relayAmplitude, double hysteresisTicksPerSec,
                   int cyclesToCollect, int cyclesToIgnore, double timeoutSeconds) {
            super("Relay Test", "Bang-bang relay test for Ku/Tu. No balls loaded; the wheel oscillates around the target.",
                    channels, ticksPerRev, voltageCompensation, nominalVoltage);
            this.targetTicksPerSec = targetTicksPerSec;
            this.relayAmplitude = relayAmplitude;
            this.hysteresisTicksPerSec = hysteresisTicksPerSec;
            this.cyclesToCollect = cyclesToCollect;
            this.cyclesToIgnore = cyclesToIgnore;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        protected List<RelayTuner.Result> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = openMotors();
            try {
                List<RelayTuner> tuners = new ArrayList<>();
                List<VelocityFilter> filters = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    tuners.add(new RelayTuner(targetTicksPerSec, relayAmplitude, hysteresisTicksPerSec,
                            cyclesToCollect, cyclesToIgnore, timeoutSeconds, 3.0));
                    filters.add(new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS));
                }
                long start = Clock.SYSTEM.nowNanos();
                long previous = start;
                while (!isStopRequested()) {
                    long now = Clock.SYSTEM.nowNanos();
                    double dt = (now - previous) / 1e9;
                    previous = now;
                    double elapsed = (now - start) / 1e9;
                    boolean allFinished = true;
                    for (int i = 0; i < channels.size(); i++) {
                        double velocity = filters.get(i).update(readVelocity(channels.get(i)), Math.max(dt, 1e-3));
                        double power = tuners.get(i).update(velocity, elapsed);
                        if (power > 1.0) {
                            power = 1.0;
                        } else if (power < 0.0) {
                            power = 0.0;
                        }
                        applyPower(channels.get(i), power);
                        String label = channels.size() > 1 ? channels.get(i).name : "wheel";
                        report(label + " target (ticks/s)", targetTicksPerSec);
                        report(label + " velocity (ticks/s)", velocity);
                        report(label + " power", power);
                        report(label + " switches", tuners.get(i).getSwitchCount());
                        if (!tuners.get(i).isFinished()) {
                            allFinished = false;
                        }
                    }
                    report("battery (V)", readBatteryVoltage());
                    telemetry.update();
                    if (allFinished) {
                        break;
                    }
                    sleep(5);
                }
                List<RelayTuner.Result> results = new ArrayList<>();
                for (RelayTuner tuner : tuners) {
                    results.add(tuner.computeResult());
                }
                return results;
            } finally {
                stopMotors();
            }
        }
    }

    /** Quasistatic stepped holds: one settled mean sample per power level (plus a descending re-check), per motor. */
    static final class QuasistaticPhase extends FlywheelPhase<List<List<SvaFit.Sample>>> {
        private final List<Double> powers;
        private final double settleSeconds;

        QuasistaticPhase(List<MotorChannel> channels, double ticksPerRev,
                         boolean voltageCompensation, double nominalVoltage,
                         List<Double> powers, double settleSeconds) {
            super("Quasistatic Ramp", "Stepped power holds for kS/kV. No balls loaded.",
                    channels, ticksPerRev, voltageCompensation, nominalVoltage);
            this.powers = new ArrayList<>(powers);
            this.settleSeconds = settleSeconds;
        }

        @Override
        protected List<List<SvaFit.Sample>> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = openMotors();
            try {
                List<List<SvaFit.Sample>> allSamples = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    allSamples.add(new ArrayList<>());
                }
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
                    List<VelocityFilter> filters = new ArrayList<>();
                    List<List<Double>> windowVelocities = new ArrayList<>();
                    List<List<Double>> windowVoltages = new ArrayList<>();
                    for (int c = 0; c < channels.size(); c++) {
                        filters.add(new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS));
                        windowVelocities.add(new ArrayList<>());
                        windowVoltages.add(new ArrayList<>());
                    }
                    long start = Clock.SYSTEM.nowNanos();
                    long previous = start;
                    while (!isStopRequested()) {
                        long now = Clock.SYSTEM.nowNanos();
                        double dt = Math.max((now - previous) / 1e9, 1e-3);
                        previous = now;
                        double elapsed = (now - start) / 1e9;
                        if (elapsed >= settleSeconds) {
                            break;
                        }
                        for (int c = 0; c < channels.size(); c++) {
                            applyPower(channels.get(c), power);
                            double velocity = filters.get(c).update(readVelocity(channels.get(c)), dt);
                            if (elapsed >= settleSeconds - 0.5) {
                                windowVelocities.get(c).add(velocity);
                                double vBatt = readBatteryVoltage();
                                if (Double.isFinite(vBatt) && vBatt > 0.0) {
                                    windowVoltages.get(c).add(vBatt);
                                }
                            }
                        }
                        report("power", power);
                        telemetry.update();
                        sleep(10);
                    }
                    if (isStopRequested()) {
                        break;
                    }
                    for (int c = 0; c < channels.size(); c++) {
                        List<Double> window = windowVelocities.get(c);
                        if (window.isEmpty()) {
                            continue;
                        }
                        double mean = 0.0;
                        for (double v : window) {
                            mean += v;
                        }
                        mean /= window.size();
                        double variance = 0.0;
                        for (double v : window) {
                            variance += (v - mean) * (v - mean);
                        }
                        double std = Math.sqrt(variance / window.size());
                        double meanVoltage = nominalVoltage;
                        if (!windowVoltages.get(c).isEmpty()) {
                            double sum = 0.0;
                            for (double v : windowVoltages.get(c)) {
                                sum += v;
                            }
                            meanVoltage = sum / windowVoltages.get(c).size();
                        }
                        // Valid only if the wheel is actually moving and settled.
                        if (mean > 50.0 && std < 0.05 * mean) {
                            allSamples.get(c).add(new SvaFit.Sample(mean, 0.0, power, meanVoltage, phaseLabel));
                        } else {
                            String label = channels.size() > 1 ? channels.get(c).name : "wheel";
                            report(label + " skipped level (unsettled/stalled)", power);
                            telemetry.update();
                        }
                    }
                }
                return allSamples;
            } finally {
                stopMotors();
            }
        }
    }

    /** Dynamic trapezoid ramps: follows a velocity profile with the preliminary controller; regressor is profile acceleration. */
    static final class DynamicPhase extends FlywheelPhase<List<List<SvaFit.Sample>>> {
        private static final double FAST_ACCEL_TICKS_PER_SEC_SQ = 2000.0;
        private static final double FAST_CEILING_TICKS_PER_SEC = 1000.0;
        private static final double SLOW_ACCEL_TICKS_PER_SEC_SQ = 100.0;
        private static final double SLOW_CEILING_TICKS_PER_SEC = 1200.0;
        private static final int FAST_REPEATS = 3;
        private static final double OVERSPEED_LIMIT_TICKS_PER_SEC = 1600.0;

        private final List<Double> preliminaryKs;
        private final List<Double> preliminaryKv;
        private final List<RelayTuner.Result> relayResults;
        private final boolean tuneIntegral;

        DynamicPhase(List<MotorChannel> channels, double ticksPerRev,
                     boolean voltageCompensation, double nominalVoltage,
                     List<Double> preliminaryKs, List<Double> preliminaryKv,
                     List<RelayTuner.Result> relayResults, boolean tuneIntegral) {
            super("Dynamic Ramps", "Trapezoid velocity ramps for kA (profile acceleration is the regressor). No balls loaded.",
                    channels, ticksPerRev, voltageCompensation, nominalVoltage);
            this.preliminaryKs = new ArrayList<>(preliminaryKs);
            this.preliminaryKv = new ArrayList<>(preliminaryKv);
            this.relayResults = new ArrayList<>(relayResults);
            this.tuneIntegral = tuneIntegral;
        }

        @Override
        protected List<List<SvaFit.Sample>> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = openMotors();
            try {
                List<DoubleFlywheelFullVelocityController> controllers = new ArrayList<>();
                List<VelocityFilter> filters = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    List<ZieglerNichols.Candidate> candidates =
                            ZieglerNichols.computeCandidates(relayResults.get(i).ku, relayResults.get(i).tuSeconds, tuneIntegral);
                    ZieglerNichols.Candidate candidate = candidates.get(2);
                    double kI = tuneIntegral ? candidate.kI : 0.0;
                    double integralLimit = kI != 0.0 ? 0.25 / Math.abs(kI) : DEFAULT_INTEGRAL_LIMIT_TICKS;
                    DoubleFlywheelFullVelocityConstants preliminary = new DoubleFlywheelFullVelocityConstants(
                            candidate.kP, kI, candidate.kD,
                            preliminaryKs.get(i), preliminaryKv.get(i), 0.0,
                            DEFAULT_TOLERANCE_TICKS_PER_SEC, integralLimit,
                            voltageCompensation, nominalVoltage);
                    DoubleFlywheelFullVelocityController controller =
                            new DoubleFlywheelFullVelocityController(preliminary);
                    controller.setVoltageSupplier(this::readBatteryVoltage);
                    controllers.add(controller);
                    filters.add(new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS));
                }

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

                List<List<SvaFit.Sample>> allSamples = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    allSamples.add(new ArrayList<>());
                }
                long start = Clock.SYSTEM.nowNanos();
                long previous = start;
                int segmentIndex = 0;
                long segmentStart = start;
                while (!isStopRequested() && segmentIndex < segments.size()) {
                    long now = Clock.SYSTEM.nowNanos();
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
                    boolean overspeed = false;
                    for (int i = 0; i < channels.size(); i++) {
                        double velocity = filters.get(i).update(readVelocity(channels.get(i)), dt);
                        controllers.get(i).setTarget(profileVelocity);
                        controllers.get(i).update(velocity, dt);
                        double power = controllers.get(i).getOutput();
                        applyPower(channels.get(i), power);
                        if (velocity > OVERSPEED_LIMIT_TICKS_PER_SEC) {
                            overspeed = true;
                        }
                        // Exclude the first 0.10 s after a segment (direction) change: the
                        // filtered velocity is still catching up and the acceleration
                        // estimate is not representative of the profile.
                        // Rest segments are not characterisation data: the wheel is only
                        // braking to a stop there (target velocity 0, still spinning
                        // from ramp-down lag, controller power negative, profile
                        // acceleration 0). Those samples contradict the plant model
                        // and drag the fitted kS negative -- in simulation 30 of them
                        // move kS from +0.15 to -0.28, matching the -0.33 seen on the
                        // robot. Skip them.
                        String segmentLabel = labels.get(segmentIndex);
                        if (segmentElapsed >= 0.10 && !segmentLabel.contains("rest")) {
                            double vBatt = readBatteryVoltage();
                            if (!Double.isFinite(vBatt) || vBatt <= 0.0) {
                                vBatt = nominalVoltage;
                            }
                            allSamples.get(i).add(new SvaFit.Sample(velocity, profileAccel, power, vBatt, segmentLabel));
                        }
                    }
                    if (overspeed) {
                        report("overspeed abort", "a wheel exceeded " + OVERSPEED_LIMIT_TICKS_PER_SEC + " ticks/sec");
                        telemetry.update();
                        break;
                    }
                    report("target (ticks/s)", profileVelocity);
                    report("segment", labels.get(segmentIndex));
                    telemetry.update();
                    sleep(5);
                }
                return allSamples;
            } finally {
                stopMotors();
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

    /** Live validation: ramp to the target, hold 5 s, ramp down; checks spin-up, overshoot, settle, and steady-state gates, per motor. */
    static final class ValidationPhase extends FlywheelPhase<List<ValidationResult>> {
        private static final double RAMP_ACCEL_TICKS_PER_SEC_SQ = 2000.0;
        private static final double HOLD_SECONDS = 5.0;
        private static final double STEADY_WINDOW_SECONDS = 3.0;

        private final List<DoubleFlywheelFullVelocityConstants> constantsList;
        private final double targetTicksPerSec;
        private final String candidateLabel;

        ValidationPhase(List<MotorChannel> channels, double ticksPerRev,
                        boolean voltageCompensation, double nominalVoltage,
                        List<DoubleFlywheelFullVelocityConstants> constantsList, double targetTicksPerSec,
                        String candidateLabel) {
            super("Validation: " + candidateLabel, "Live test of candidate " + candidateLabel + " at the target RPM. No balls loaded.",
                    channels, ticksPerRev, voltageCompensation, nominalVoltage);
            this.constantsList = new ArrayList<>(constantsList);
            this.targetTicksPerSec = targetTicksPerSec;
            this.candidateLabel = candidateLabel;
        }

        @Override
        protected List<ValidationResult> runTuningOpMode() throws InterruptedException {
            waitForStart();
            List<DcMotorEx> motors = openMotors();
            try {
                List<DoubleFlywheelFullVelocityController> controllers = new ArrayList<>();
                List<VelocityFilter> filters = new ArrayList<>();
                List<Double> tolerances = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    DoubleFlywheelFullVelocityController controller =
                            new DoubleFlywheelFullVelocityController(constantsList.get(i));
                    controller.setVoltageSupplier(this::readBatteryVoltage);
                    controllers.add(controller);
                    filters.add(new VelocityFilter(VELOCITY_FILTER_TAU_SECONDS));
                    tolerances.add(Math.max(constantsList.get(i).tolerance, 30.0));
                }
                double rampSeconds = targetTicksPerSec / RAMP_ACCEL_TICKS_PER_SEC_SQ;
                double totalSeconds = rampSeconds + HOLD_SECONDS + rampSeconds;

                List<List<Double>> errorsPerMotor = new ArrayList<>();
                List<List<Double>> timesPerMotor = new ArrayList<>();
                List<Double> maxVelocities = new ArrayList<>();
                List<Double> firstBandTimes = new ArrayList<>();
                List<Double> lastOutOfBandTimes = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    errorsPerMotor.add(new ArrayList<>());
                    timesPerMotor.add(new ArrayList<>());
                    maxVelocities.add(0.0);
                    firstBandTimes.add(Double.NaN);
                    lastOutOfBandTimes.add(Double.NaN);
                }
                long start = Clock.SYSTEM.nowNanos();
                long previous = start;
                while (!isStopRequested()) {
                    long now = Clock.SYSTEM.nowNanos();
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
                    for (int i = 0; i < channels.size(); i++) {
                        double velocity = filters.get(i).update(readVelocity(channels.get(i)), dt);
                        controllers.get(i).setTarget(profileVelocity);
                        controllers.get(i).update(velocity, dt);
                        applyPower(channels.get(i), controllers.get(i).getOutput());
                        if (velocity > maxVelocities.get(i)) {
                            maxVelocities.set(i, velocity);
                        }
                        if (elapsed >= rampSeconds && elapsed < rampSeconds + HOLD_SECONDS) {
                            double error = targetTicksPerSec - velocity;
                            errorsPerMotor.get(i).add(error);
                            timesPerMotor.get(i).add(elapsed - rampSeconds);
                            if (Math.abs(error) <= tolerances.get(i)) {
                                if (Double.isNaN(firstBandTimes.get(i))) {
                                    firstBandTimes.set(i, elapsed - rampSeconds);
                                }
                            } else {
                                lastOutOfBandTimes.set(i, elapsed - rampSeconds);
                            }
                        }
                    }
                    report("target (ticks/s)", profileVelocity);
                    telemetry.update();
                    sleep(5);
                }
                List<ValidationResult> results = new ArrayList<>();
                for (int i = 0; i < channels.size(); i++) {
                    List<Double> errors = errorsPerMotor.get(i);
                    List<Double> times = timesPerMotor.get(i);
                    double tolerance = tolerances.get(i);
                    double spinUp = Double.isNaN(firstBandTimes.get(i)) ? Double.NaN : firstBandTimes.get(i);
                    double overshoot = targetTicksPerSec > 0.0 ? Math.max(0.0, maxVelocities.get(i) / targetTicksPerSec - 1.0) : Double.NaN;
                    // Settle: 0.5 s of continuous in-band after the last out-of-band sample; NaN if never settled.
                    double settle = Double.NaN;
                    if (!Double.isNaN(lastOutOfBandTimes.get(i))) {
                        settle = lastOutOfBandTimes.get(i) + 0.5;
                        if (settle > HOLD_SECONDS) {
                            settle = Double.NaN;
                        }
                    } else if (!errors.isEmpty()) {
                        settle = 0.0;
                    }
                    // Steady-state window: last STEADY_WINDOW_SECONDS of the hold.
                    List<Double> steadyErrors = new ArrayList<>();
                    for (int j = 0; j < errors.size(); j++) {
                        if (times.get(j) >= HOLD_SECONDS - STEADY_WINDOW_SECONDS) {
                            steadyErrors.add(errors.get(j));
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
                    for (int j = 0; j < errors.size(); j++) {
                        if (times.get(j) >= settleStart && times.get(j) <= settleStart + 2.0) {
                            double e = errors.get(j);
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
                    results.add(new ValidationResult(passed, spinUp, overshoot, settle, meanError, std,
                            maxExcursion, crossings, gateDetail));
                }
                return results;
            } finally {
                stopMotors();
            }
        }
    }
}
