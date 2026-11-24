package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.math.Pose;
import org.junit.Test;

public class AutoAimerCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void correctedPoseFallsBackToProvidedPose() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        Pose fallback = new Pose(12.0, 34.0, Math.PI / 4.0);

        Pose corrected = calculator.getCorrectedPoseOrFallback(fallback);

        assertEquals(fallback.x(), corrected.x(), EPSILON);
        assertEquals(fallback.y(), corrected.y(), EPSILON);
        assertEquals(fallback.heading(), corrected.heading(), EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void correctedPoseFallbackRejectsNull() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        calculator.getCorrectedPoseOrFallback(null);
    }

    @Test
    public void turningPowerIsNegativeWhenHeadingIsAboveTarget() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        calculator.updatePedro(
                new com.pedropathing.math.Pose(0.0, 60.0, 0.0),
                new com.pedropathing.math.Velocity(0.0, 0.0, 0.0),
                0.0);

        // Target heading is 0 (goal straight ahead at (60, 60)). Heading
        // 10 deg above it must give a negative error and negative power;
        // Angle.normalize (unsigned) would read it as +350 deg and turn the
        // wrong way at full power.
        assertEquals(-0.08726646259971647, calculator.calculateTurningPower(Math.toRadians(10.0)), EPSILON);
    }

    @Test
    public void updatePedroAppliesShootOnMoveHeadingOffset() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        calculator.updatePedro(
                new com.pedropathing.math.Pose(20.0, 60.0, 0.0),
                new com.pedropathing.math.Velocity(0.0, 30.0, 0.0),
                0.0);

        // Raw bearing to the goal is 0. Driving 30 in/s sideways, the ball
        // inherits that velocity, so the aim heading rotates by
        // atan2(30, Vreq) with Vreq = 199.347 * cos(62.7 deg) = 91.43 in/s:
        // 0.31705 rad (18.17 deg).
        assertEquals(0.3170499837428561, calculator.getTargetHeading(), EPSILON);
    }
}
