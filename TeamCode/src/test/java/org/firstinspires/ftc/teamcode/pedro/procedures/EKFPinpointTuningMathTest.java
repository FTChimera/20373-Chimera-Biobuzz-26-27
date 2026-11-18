package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class EKFPinpointTuningMathTest {
    @Test
    public void varianceUsesUnbiasedSampleVariance() {
        assertEquals(5.0 / 3.0, EKFPinpointTuningMath.variance(Arrays.asList(1.0, 2.0, 3.0, 4.0)), 1e-9);
    }

    @Test
    public void emptySamplesProduceZeroStatistics() {
        assertEquals(0.0, EKFPinpointTuningMath.mean(Collections.emptyList()), 0.0);
        assertEquals(0.0, EKFPinpointTuningMath.variance(Collections.singletonList(1.0)), 0.0);
    }

    @Test
    public void headingRingFrequencyIsAboveOneHertzForTheMeasuredTuning() {
        // The value the tuner measured on the robot (9.08 rad/s^2 with 1 degree
        // heading noise) makes the heading estimator ring at ~2.6 Hz, which the
        // follower turns into left-right swaying.
        double ring = EKFPinpointTuningMath.ringFrequencyHz(
                9.08384779, Math.toRadians(1.0), 0.02);
        assertTrue(ring > 2.0 && ring < 3.0);
    }

    @Test
    public void headingRingFrequencyIsBelowOneHertzForTheRecommendedTuning() {
        double ring = EKFPinpointTuningMath.ringFrequencyHz(
                1.0, Math.toRadians(1.0), 0.02);
        assertTrue(ring > 0.5 && ring < 1.0);
    }

    @Test
    public void angularCapKeepsRingFrequencyAtOrBelowOneHertz() {
        double cap = EKFPinpointTuningMath.maxAccelerationStandardDeviation(
                Math.toRadians(1.0), 0.02, 1.0);
        assertTrue(cap > 1.0 && cap < 2.0);
        assertTrue(EKFPinpointTuningMath.ringFrequencyHz(cap, Math.toRadians(1.0), 0.02) <= 1.0);
    }

    @Test
    public void linearCapKeepsRingFrequencyAtOrBelowOneHertz() {
        double cap = EKFPinpointTuningMath.maxAccelerationStandardDeviation(
                0.1, 0.02, 1.0);
        assertTrue(cap > 5.0 && cap < 10.0);
        assertTrue(EKFPinpointTuningMath.ringFrequencyHz(cap, 0.1, 0.02) <= 1.0);
    }

    @Test
    public void smallProcessNoiseDoesNotRing() {
        assertTrue(EKFPinpointTuningMath.ringFrequencyHz(0.1, Math.toRadians(1.0), 0.02) < 1.0);
    }
}
