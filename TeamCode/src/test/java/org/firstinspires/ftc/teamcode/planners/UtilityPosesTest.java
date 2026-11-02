package org.firstinspires.ftc.teamcode.planners;

import static org.junit.Assert.assertFalse;

import org.junit.Test;

public class UtilityPosesTest {
    @Test
    public void placeholderCornerRezeroesAreUnconfigured() {
        assertFalse(UtilityPoses.isCornerLoadingAreaConfigured());
        assertFalse(UtilityPoses.isCornerGardenAreaConfigured());
    }
}
