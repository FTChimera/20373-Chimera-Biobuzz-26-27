package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Pins the shoot-on-the-move compensation in {@link ProjectileShotCalc}:
 * the heading offset rotates against the robot's sideways drift, and the
 * wheel speed uses the full launch-vector magnitude |L|, not just the
 * parallel component.
 */
public class ProjectileShotCalcTest {
    private static final double EPSILON = 1e-9;

    // 40 in shot: launch speed 199.347 in/s, Vreq = 91.43 in/s horizontal.
    private static final double LAUNCH_SPEED_40IN = 199.34686788831274;
    private static final double TICKS_40IN = 807.5973669097598;

    @Test
    public void compensateRotatesAgainstSidewaysDrift() {
        double[] comp = ProjectileShotCalc.compensate(
                LAUNCH_SPEED_40IN, 62.7, 0.0, 30.0, 1.0, 0.0);

        // Driving +y at 30 in/s while shooting along +x: L = Vreq*s - v
        // points -18.17 deg off the bearing, so the offset is negative.
        assertEquals(91.43035214481822, comp[0], EPSILON);
        assertEquals(-0.3170499837428561, comp[1], EPSILON);
    }

    @Test
    public void compensateOffsetMirrorsWithDriftDirection() {
        double[] comp = ProjectileShotCalc.compensate(
                LAUNCH_SPEED_40IN, 62.7, 0.0, -30.0, 1.0, 0.0);

        assertEquals(91.43035214481822, comp[0], EPSILON);
        assertEquals(0.3170499837428561, comp[1], EPSILON);
    }

    @Test
    public void compensateHeadOnHasNoHeadingOffset() {
        double[] comp = ProjectileShotCalc.compensate(
                LAUNCH_SPEED_40IN, 62.7, 30.0, 0.0, 1.0, 0.0);

        assertEquals(61.43035214481822, comp[0], EPSILON);
        assertEquals(0.0, comp[1], EPSILON);
    }

    @Test
    public void compensateTicksUsesFullLaunchVectorMagnitude() {
        // Sideways 30 in/s: |L| = hypot(91.43, 30) = 96.23 in/s, so the
        // wheel must spin FASTER than the stationary 807.6 ticks/s — the
        // parallel-only formula gave 807.6 (5% slow, a short shot).
        assertEquals(849.959999894053,
                ProjectileShotCalc.compensateTicks(TICKS_40IN, 0.0, 30.0, 1.0, 0.0),
                EPSILON);
    }

    @Test
    public void compensateTicksHeadOnMatchesParallelOnlyCase() {
        // No perpendicular component: |L| is just the parallel remainder.
        assertEquals(542.6096419481656,
                ProjectileShotCalc.compensateTicks(TICKS_40IN, 30.0, 0.0, 1.0, 0.0),
                EPSILON);
    }

    @Test
    public void compensateTicksReturnsZeroWhenOverSpeed() {
        assertEquals(0.0,
                ProjectileShotCalc.compensateTicks(TICKS_40IN, 100.0, 0.0, 1.0, 0.0),
                EPSILON);
    }
}
