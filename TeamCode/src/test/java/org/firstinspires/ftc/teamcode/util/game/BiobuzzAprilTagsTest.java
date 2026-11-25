package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.AllianceColor;

import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

/**
 * Pins the official BIOBUZZ AprilTag table (36h11, one 4-tag cluster per
 * cell). The RED clusters were 0-7 (a DECODE carry-over) until 2026-11-24,
 * which matched no BIOBUZZ tag — a RED robot's Limelight could see tags
 * 31, 32, 33 fresh and clear while {@code Vision} looked for 0-7, so the
 * hive was never "seen" and the tree auto always switched sides.
 */
public class BiobuzzAprilTagsTest {

    private static Set<Integer> union(int[]... tagSets) {
        Set<Integer> ids = new HashSet<>();
        for (int[] tags : tagSets) {
            for (int id : tags) {
                ids.add(id);
            }
        }
        return ids;
    }

    @Test
    public void redClustersMatchOfficialBiobuzzTable() {
        assertArrayEquals(new int[]{30, 31, 32, 33}, BiobuzzAprilTags.RED_OPPOSITE.getTags());
        assertArrayEquals(new int[]{34, 35, 36, 37}, BiobuzzAprilTags.RED_AUDIENCE.getTags());
    }

    @Test
    public void blueClustersMatchOfficialBiobuzzTable() {
        assertArrayEquals(new int[]{38, 39, 40, 41}, BiobuzzAprilTags.BLUE_AUDIENCE.getTags());
        assertArrayEquals(new int[]{42, 43, 44, 45}, BiobuzzAprilTags.BLUE_OPPOSITE.getTags());
    }

    @Test
    public void redInstanceSeesObservedTags31To33AndNoDecodeIds() {
        BiobuzzAprilTags tags = new BiobuzzAprilTags(AllianceColor.RED);
        Set<Integer> redIds = union(tags.alliance.getTags(), tags.opposite.getTags());
        // Tag 31 is the one Shlok observed live on the red hive.
        assertTrue(redIds.contains(31));
        assertTrue(redIds.contains(30));
        assertTrue(redIds.contains(32));
        assertTrue(redIds.contains(33));
        for (int decodeId = 0; decodeId <= 7; decodeId++) {
            assertFalse(redIds.contains(decodeId));
        }
    }

    @Test
    public void blueInstanceSeesOnlyBlueIds() {
        BiobuzzAprilTags tags = new BiobuzzAprilTags(AllianceColor.BLUE);
        Set<Integer> blueIds = union(tags.alliance.getTags(), tags.opposite.getTags());
        for (int id = 38; id <= 45; id++) {
            assertTrue(blueIds.contains(id));
        }
        for (int id = 30; id <= 37; id++) {
            assertFalse(blueIds.contains(id));
        }
    }
}
