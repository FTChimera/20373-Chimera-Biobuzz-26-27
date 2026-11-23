package org.firstinspires.ftc.teamcode.opmodes;

/** Pure arbitration for the TeleOp's held overrides and normal feed requests. */
final class TeleOpOuttakeState {
    enum ShooterMode { OFF, SMART, PANIC, REVERSE }
    enum FeedMode { STOP, FAST, SLOW, PANIC, EJECT }

    ShooterMode shooter = ShooterMode.OFF;
    FeedMode feed = FeedMode.STOP;
    private boolean manualFastStarted;

    void update(boolean reverse, boolean panic, boolean shooterEnabled,
                boolean fast, boolean slow, boolean autoFire, boolean eject,
                boolean upToSpeed, boolean headingReady) {
        ShooterMode previous = shooter;
        shooter = reverse ? ShooterMode.REVERSE
                : panic ? ShooterMode.PANIC
                : shooterEnabled ? ShooterMode.SMART : ShooterMode.OFF;
        // RT is a manual burst: speed-gated on entry, not interrupted by each
        // ball's speed dip. Explicit shutdown or an override ends that burst.
        if (!fast || shooter != ShooterMode.SMART || eject
                || previous != ShooterMode.SMART) {
            manualFastStarted = false;
        }
        if (fast && shooter == ShooterMode.SMART && !eject
                && previous == ShooterMode.SMART && upToSpeed) {
            manualFastStarted = true;
        }

        if (shooter == ShooterMode.REVERSE) feed = FeedMode.STOP;
        else if (shooter == ShooterMode.PANIC) feed = FeedMode.PANIC;
        else if (eject) feed = FeedMode.EJECT;
        else if (shooter == ShooterMode.SMART && previous == ShooterMode.SMART
                && autoFire && upToSpeed && headingReady) feed = FeedMode.FAST;
        else if (slow) feed = FeedMode.SLOW;
        else if (manualFastStarted) feed = FeedMode.FAST;
        else feed = FeedMode.STOP;
    }

    static boolean headingReady(double target, double heading, double toleranceDegrees) {
        if (!Double.isFinite(toleranceDegrees) || toleranceDegrees < 0
                || toleranceDegrees > 180) {
            throw new IllegalArgumentException("Heading tolerance must be between 0 and 180 degrees.");
        }
        if (!Double.isFinite(target) || !Double.isFinite(heading)) return false;
        double error = Math.atan2(Math.sin(target - heading), Math.cos(target - heading));
        return Math.abs(error) <= Math.toRadians(toleranceDegrees);
    }
}
