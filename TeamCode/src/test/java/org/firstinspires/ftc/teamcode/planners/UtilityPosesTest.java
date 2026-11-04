package org.firstinspires.ftc.teamcode.planners;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class UtilityPosesTest {
    @Test
    public void cornerRezeroesUseConfiguredRedPoses() {
        assertTrue(UtilityPoses.isCornerLoadingAreaConfigured());
        assertTrue(UtilityPoses.isCornerGardenAreaConfigured());
    }
}
