package org.firstinspires.ftc.teamcode.robotDev.sim;

/** Pure, latched checks of sampled SDK outputs; not ball or physical-servo feedback. */
final class IntakeSequenceMetrics {
    static final double GRACE_S = 0.15, MAX_GAP_S = 0.10;
    static final double POWER_TOLERANCE = 0.005, SERVO_TOLERANCE = 0.005;
    static final double MOTION_TPS = 5, MOTION_TICKS = 1, TOTAL_S = 5.5;
    private static final double EPS = 1e-9;

    enum Phase {
        STOP_STOW(0, 0.5), DEPLOY_ONLY(0.5, 1), INTAKE(1, 3),
        TRANSFER(3, 5), FINAL_STOP(5, TOTAL_S);
        final double start, end;
        Phase(double start, double end) { this.start = start; this.end = end; }
        boolean deployed() { return this != STOP_STOW && this != FINAL_STOP; }
        boolean intakeOn() { return this == INTAKE || this == TRANSFER; }
        double minimumSpan() { return end - start - GRACE_S - MAX_GAP_S; }
        int minimumSamples() { return intakeOn() ? 10 : 4; }
    }

    static Phase phaseAt(double elapsed) {
        if (!finite(elapsed) || elapsed < 0) throw new IllegalArgumentException("Invalid sequence time");
        for (Phase phase : Phase.values()) if (elapsed < phase.end) return phase;
        return Phase.FINAL_STOP;
    }

    static boolean finite(double... values) {
        for (double value : values) if (Double.isNaN(value) || Double.isInfinite(value)) return false;
        return true;
    }

    static boolean matches(double actual, double expected, double tolerance) {
        return finite(actual, expected, tolerance) && tolerance >= 0
                && actual >= expected - tolerance && actual <= expected + tolerance;
    }

    final int[] samples = new int[Phase.values().length];
    final double[] first = new double[Phase.values().length];
    final double[] last = new double[Phase.values().length];
    final double[] maxIntakeError = new double[Phase.values().length];
    final double[] maxTransferError = new double[Phase.values().length];
    final double[] maxServoError = new double[Phase.values().length];
    private final double intakePower, transferPower, deployed, stowed;
    private double previousTime = Double.NaN;
    private Phase previousPhase;
    private boolean intakeMoved, transferMoved, haveIntakeEncoder, haveTransferEncoder;
    private double intakeMin, intakeMax, transferMin, transferMax;
    boolean passed;
    String failure;

    IntakeSequenceMetrics(double intakePower, double transferPower, double deployed, double stowed) {
        if (!finite(intakePower, transferPower, deployed, stowed)
                || Math.abs(intakePower) > 1 || Math.abs(transferPower) > 1
                || intakePower == 0 || transferPower == 0
                || deployed < 0 || deployed > 1 || stowed < 0 || stowed > 1
                || Math.abs(deployed - stowed) <= 2 * SERVO_TOLERANCE) {
            throw new IllegalArgumentException("Invalid mechanism expectations");
        }
        this.intakePower = intakePower;
        this.transferPower = transferPower;
        this.deployed = deployed;
        this.stowed = stowed;
        for (int i = 0; i < first.length; i++) first[i] = last[i] = Double.NaN;
    }

    double expectedIntake(Phase phase) { return phase.intakeOn() ? intakePower : 0; }
    double expectedTransfer(Phase phase) { return phase == Phase.TRANSFER ? transferPower : 0; }
    double expectedServo(Phase phase) { return phase.deployed() ? deployed : stowed; }
    boolean intakeMoved() { return intakeMoved; }
    boolean transferMoved() { return transferMoved; }
    double span(Phase phase) {
        int i = phase.ordinal();
        return samples[i] == 0 ? 0 : last[i] - first[i];
    }

    void sample(double time, double intake, double transfer, double servo,
                double intakeVelocity, double transferVelocity, double intakeEncoder, double transferEncoder) {
        if (failure != null || passed) return;
        if (!finite(time, intake, transfer, servo, intakeVelocity, transferVelocity,
                intakeEncoder, transferEncoder) || time < 0
                || Math.abs(intake) > 1 || Math.abs(transfer) > 1 || servo < 0 || servo > 1) {
            fail("nonfinite/out-of-range observation"); return;
        }
        if (Double.isNaN(previousTime) ? time > MAX_GAP_S + EPS
                : time <= previousTime || time - previousTime > MAX_GAP_S + EPS) {
            fail("nonmonotonic time or tick gap > 0.10 s"); return;
        }
        Phase phase = phaseAt(time);
        if (previousPhase == null && phase != Phase.STOP_STOW
                || previousPhase != null && phase.ordinal() > previousPhase.ordinal() + 1) {
            fail("skipped phase"); return;
        }
        if (previousPhase != null && phase != previousPhase && !covered(previousPhase)) {
            fail("insufficient coverage: " + previousPhase); return;
        }
        previousTime = time;
        previousPhase = phase;
        int i = phase.ordinal();
        // Grace ignores only matching/coverage, never invalid values or missing ticks.
        if (time - phase.start + EPS >= GRACE_S && time < phase.end) {
            double ie = Math.abs(intake - expectedIntake(phase));
            double te = Math.abs(transfer - expectedTransfer(phase));
            double se = Math.abs(servo - expectedServo(phase));
            maxIntakeError[i] = Math.max(maxIntakeError[i], ie);
            maxTransferError[i] = Math.max(maxTransferError[i], te);
            maxServoError[i] = Math.max(maxServoError[i], se);
            if (!matches(intake, expectedIntake(phase), POWER_TOLERANCE)
                    || !matches(transfer, expectedTransfer(phase), POWER_TOLERANCE)
                    || !matches(servo, expectedServo(phase), SERVO_TOLERANCE)) {
                fail("applied power/SDK servo mismatch: " + phase); return;
            }
            if (samples[i]++ == 0) first[i] = time;
            last[i] = time;
            if (phase.intakeOn()) {
                if (!haveIntakeEncoder) { intakeMin = intakeMax = intakeEncoder; haveIntakeEncoder = true; }
                intakeMin = Math.min(intakeMin, intakeEncoder);
                intakeMax = Math.max(intakeMax, intakeEncoder);
                intakeMoved |= Math.abs(intakeVelocity) >= MOTION_TPS || intakeMax - intakeMin >= MOTION_TICKS;
            }
            if (phase == Phase.TRANSFER) {
                if (!haveTransferEncoder) { transferMin = transferMax = transferEncoder; haveTransferEncoder = true; }
                transferMin = Math.min(transferMin, transferEncoder);
                transferMax = Math.max(transferMax, transferEncoder);
                transferMoved |= Math.abs(transferVelocity) >= MOTION_TPS || transferMax - transferMin >= MOTION_TICKS;
            }
        }
        if (time >= TOTAL_S) {
            for (Phase required : Phase.values()) {
                if (!covered(required)) { fail("insufficient coverage: " + required); return; }
            }
            if (!intakeMoved || !transferMoved) { fail("both motors must show measured motion"); return; }
            // The completion tick is still an observed STOP/STOW check.
            if (!matches(intake, 0, POWER_TOLERANCE) || !matches(transfer, 0, POWER_TOLERANCE)
                    || !matches(servo, stowed, SERVO_TOLERANCE)) {
                fail("completion STOP/STOW mismatch"); return;
            }
            passed = true;
        }
    }

    private boolean covered(Phase phase) {
        return samples[phase.ordinal()] >= phase.minimumSamples() && span(phase) + EPS >= phase.minimumSpan();
    }

    private void fail(String reason) { failure = reason; }
}
