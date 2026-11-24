package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.Test;

/**
 * Tests for the strict cluster decision behind the tree auto's camera
 * check ({@link Vision#inferHiveStateFromLatestFrame()}): the dominant
 * cluster names the hive state, and anything else is "no evidence" — never
 * a carried-over state. The area-summing loop itself is shared with the
 * carried-over inference, so it is pinned here through its pure decision.
 */
public class VisionHiveInferenceTest {

    @Test
    public void redAllianceClusterDominantIsUntipped() {
        assertEquals(Field.HiveState.UNTIPPED,
                Vision.inferHiveStateFromClusterAreas(2.0, 0.5, AllianceColor.RED));
    }

    @Test
    public void redOppositeClusterDominantIsTipped() {
        assertEquals(Field.HiveState.TIPPED,
                Vision.inferHiveStateFromClusterAreas(0.5, 2.0, AllianceColor.RED));
    }

    @Test
    public void blueClusterMappingIsMirrored() {
        assertEquals(Field.HiveState.TIPPED,
                Vision.inferHiveStateFromClusterAreas(2.0, 0.5, AllianceColor.BLUE));
        assertEquals(Field.HiveState.UNTIPPED,
                Vision.inferHiveStateFromClusterAreas(0.5, 2.0, AllianceColor.BLUE));
    }

    @Test
    public void noTagsIsNoEvidence() {
        assertNull(Vision.inferHiveStateFromClusterAreas(0.0, 0.0, AllianceColor.RED));
        assertNull(Vision.inferHiveStateFromClusterAreas(0.0, 0.0, AllianceColor.BLUE));
    }

    @Test
    public void clustersWithinMarginAreAmbiguous() {
        // HIVE_CLUSTER_AREA_MARGIN is 0.05: a 0.04 difference is too close
        // to call, so the tree must not switch on it — and must not fall
        // back to a state carried over from an earlier frame.
        assertNull(Vision.inferHiveStateFromClusterAreas(1.0, 1.0, AllianceColor.RED));
        assertNull(Vision.inferHiveStateFromClusterAreas(1.0, 1.04, AllianceColor.RED));
        assertNull(Vision.inferHiveStateFromClusterAreas(1.04, 1.0, AllianceColor.RED));
    }

    @Test
    public void differenceAtMarginDecides() {
        // Exactly at the margin is not "within" it, so the dominant cluster
        // decides.
        assertEquals(Field.HiveState.UNTIPPED,
                Vision.inferHiveStateFromClusterAreas(1.05, 1.0, AllianceColor.RED));
        assertEquals(Field.HiveState.TIPPED,
                Vision.inferHiveStateFromClusterAreas(1.0, 1.05, AllianceColor.RED));
    }
}
