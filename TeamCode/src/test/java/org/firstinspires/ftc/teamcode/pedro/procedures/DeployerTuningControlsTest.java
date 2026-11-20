package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DeployerTuningControlsTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void leftTriggerSlowsAdjustmentStep() {
        assertEquals(0.05, DeployerTuningControls.step(0.0), EPSILON);
        assertEquals(0.025, DeployerTuningControls.step(0.5), EPSILON);
        assertEquals(0.0, DeployerTuningControls.step(1.0), EPSILON);
    }

    @Test
    public void adjustmentClipsToServoRange() {
        assertEquals(1.0, DeployerTuningControls.adjust(0.99, true, false, 0.0), EPSILON);
        assertEquals(0.0, DeployerTuningControls.adjust(0.01, false, true, 0.0), EPSILON);
    }

    @Test
    public void oppositeDirectionsCancel() {
        assertEquals(0.4, DeployerTuningControls.adjust(0.4, true, true, 0.0), EPSILON);
    }

    @Test
    public void translationAlignsConfiguredGroundWithMeasuredGround() {
        assertEquals(-0.03, DeployerTuningControls.translationFactor(0.19, 0.22), EPSILON);
    }
}
