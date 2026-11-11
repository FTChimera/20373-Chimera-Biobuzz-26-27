package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.assertEquals;

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
}
