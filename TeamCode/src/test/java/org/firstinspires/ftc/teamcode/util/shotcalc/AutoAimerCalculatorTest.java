package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.junit.Test;

public class AutoAimerCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void correctedPoseFallsBackToProvidedPose() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        Pose2d fallback = new Pose2d(12.0, 34.0, Math.PI / 4.0);

        Pose2d corrected = calculator.getCorrectedPoseOrFallback(fallback);

        assertEquals(fallback.x(), corrected.x(), EPSILON);
        assertEquals(fallback.y(), corrected.y(), EPSILON);
        assertEquals(fallback.heading(), corrected.heading(), EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void correctedPoseFallbackRejectsNull() {
        AutoAimerCalculator calculator = new AutoAimerCalculator(AllianceColor.RED);
        calculator.getCorrectedPoseOrFallback(null);
    }
}
