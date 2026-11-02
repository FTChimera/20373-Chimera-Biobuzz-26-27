package org.firstinspires.ftc.teamcode.util.shotcalc;

import org.firstinspires.ftc.teamcode.util.game.Ball;

/**
 * A linear, unbounded model matching the slope of the default shot table.
 * It is useful for testing extrapolation; match-calibrated shots use
 * {@link TableShotCalc}.
 */
public class IdealShotCalc implements ShotCalculator {
    public static final double POLLEN_BASE_TICKS_PER_SECOND = 200.0;
    public static final double NECTAR_BASE_TICKS_PER_SECOND = 400.0;
    public static final double TICKS_PER_SECOND_PER_INCH = 20.0;

    @Override
    public ShotSolution calculateShot(ShotContext context) {
        if (context == null) {
            throw new IllegalArgumentException("Shot context is required.");
        }
        ShotContext ctx = context.copy();
        ctx.validate();
        if (ctx.sotm) {
            ctx.robotPose = ctx.poseAtRelease();
        }

        double baseSpeed = ctx.transferBallCol.ball == Ball.NECTAR
                ? NECTAR_BASE_TICKS_PER_SECOND
                : POLLEN_BASE_TICKS_PER_SECOND;
        return new ShotSolution(baseSpeed + TICKS_PER_SECOND_PER_INCH * ctx.distanceToTarget());
    }
}
