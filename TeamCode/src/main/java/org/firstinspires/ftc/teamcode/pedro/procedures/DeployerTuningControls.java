package org.firstinspires.ftc.teamcode.pedro.procedures;

public final class DeployerTuningControls {
    public static final double POSITION_STEP_DIVISOR = 20.0;
    public static final double MIN_POSITION = 0.0;
    public static final double MAX_POSITION = 1.0;
    public static final long ADJUSTMENT_INTERVAL_MILLIS = 100;

    private DeployerTuningControls() {
    }

    static double step(double leftTrigger) {
        double clippedTrigger = clip(leftTrigger, 0.0, 1.0);
        return (1.0 - clippedTrigger) / POSITION_STEP_DIVISOR;
    }

    static double adjust(double position, boolean increase, boolean decrease, double leftTrigger) {
        double direction = (increase ? 1.0 : 0.0) - (decrease ? 1.0 : 0.0);
        return clip(position + direction * step(leftTrigger), MIN_POSITION, MAX_POSITION);
    }

    static double translationFactor(double measuredGroundPosition, double configuredGroundPosition) {
        return measuredGroundPosition - configuredGroundPosition;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
