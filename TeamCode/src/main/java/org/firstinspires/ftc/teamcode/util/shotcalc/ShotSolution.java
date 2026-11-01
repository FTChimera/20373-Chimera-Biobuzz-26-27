package org.firstinspires.ftc.teamcode.util.shotcalc;

public class ShotSolution {
    /** Motor encoder velocity in ticks per second. */
    public final double flywheelTicksPerSecond;

    public ShotSolution(double flywheelTicksPerSecond) {
        if (!Double.isFinite(flywheelTicksPerSecond) || flywheelTicksPerSecond < 0.0) {
            throw new IllegalArgumentException("Flywheel velocity must be finite and nonnegative.");
        }
        this.flywheelTicksPerSecond = flywheelTicksPerSecond;
    }
}
