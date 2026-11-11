package org.firstinspires.ftc.teamcode.util.shotcalc;

public class ShotSolution {
    /** Main flywheel motor encoder velocity in ticks per second. */
    public final double flywheelTicksPerSecond;
    /** Counter roller motor encoder velocity in ticks per second. */
    public final double counterRollerTicksPerSecond;

    public ShotSolution(double flywheelTicksPerSecond, double counterRollerTicksPerSecond) {
        if (!Double.isFinite(flywheelTicksPerSecond) || flywheelTicksPerSecond < 0.0
                || !Double.isFinite(counterRollerTicksPerSecond) || counterRollerTicksPerSecond < 0.0) {
            throw new IllegalArgumentException("Flywheel velocities must be finite and nonnegative.");
        }
        this.flywheelTicksPerSecond = flywheelTicksPerSecond;
        this.counterRollerTicksPerSecond = counterRollerTicksPerSecond;
    }
}
