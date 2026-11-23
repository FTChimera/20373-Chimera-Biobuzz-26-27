package org.firstinspires.ftc.teamcode.planners.tree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.Test;

import java.util.EnumSet;

/**
 * Tests for the tree auto's pure logic: the hive tip model and the leg
 * selection policy. The selection traces simulate the blackboard evolving
 * as legs complete (a leg's shoot may land on the other side when the
 * camera check forces a switch).
 */
public class TreeStateTest {

    /** Simulates blackboard state as legs/repositions complete. */
    private static class Sim {
        Position position = Position.GARDEN_START;
        int cycles = 0;
        EnumSet<IntakeArea> done = EnumSet.noneOf(IntakeArea.class);

        TreeState.Selection next() {
            return TreeState.selectLeg(position, cycles, done);
        }

        /** Completes a leg shooting from its planned side. */
        void applyLeg(TreeState.LegId leg) {
            applyLeg(leg, leg.to());
        }

        /** Completes a leg shooting from {@code shotFrom} (camera switch). */
        void applyLeg(TreeState.LegId leg, Position shotFrom) {
            cycles++;
            if (leg.intakeArea() != null) done.add(leg.intakeArea());
            position = shotFrom;
        }

        void applyReposition() {
            position = TreeState.otherShootPosition(position);
        }
    }

    @Test
    public void tipModelMatchesSpecTable() {
        assertEquals(Field.HiveState.UNTIPPED, TreeState.predictHiveState(0));
        assertEquals(Field.HiveState.TIPPED, TreeState.predictHiveState(1));
        assertEquals(Field.HiveState.TIPPED, TreeState.predictHiveState(2));
        assertEquals(Field.HiveState.UNTIPPED, TreeState.predictHiveState(3));
        assertEquals(Field.HiveState.UNTIPPED, TreeState.predictHiveState(4));
        assertEquals(Field.HiveState.TIPPED, TreeState.predictHiveState(5));
        assertEquals(Field.HiveState.TIPPED, TreeState.predictHiveState(6));
        assertEquals(Field.HiveState.UNTIPPED, TreeState.predictHiveState(7));
    }

    @Test
    public void optimalSideFollowsPredictedState() {
        assertEquals(Position.GARDEN_SHOOT, TreeState.optimalShootPosition(0));
        assertEquals(Position.LOADING_SHOOT, TreeState.optimalShootPosition(1));
        assertEquals(Position.LOADING_SHOOT, TreeState.optimalShootPosition(2));
        assertEquals(Position.GARDEN_SHOOT, TreeState.optimalShootPosition(3));
        assertEquals(Position.GARDEN_SHOOT, TreeState.optimalShootPosition(4));
        assertEquals(Position.LOADING_SHOOT, TreeState.optimalShootPosition(5));
    }

    @Test
    public void perfectPredictionReproducesGardenSoloOrder() {
        Sim sim = new Sim();

        assertEquals(TreeState.Selection.SHOOT_PRELOADS, sim.next());
        sim.applyLeg(TreeState.LegId.SHOOT_PRELOADS);

        assertEquals(TreeState.Selection.GARDEN_TO_LOADING, sim.next());
        sim.applyLeg(TreeState.LegId.GARDEN_TO_LOADING);

        assertEquals(TreeState.Selection.NORTH_FLOWER, sim.next());
        sim.applyLeg(TreeState.LegId.NORTH_FLOWER);

        assertEquals(TreeState.Selection.LOADING_TO_GARDEN, sim.next());
        sim.applyLeg(TreeState.LegId.LOADING_TO_GARDEN);

        assertEquals(TreeState.Selection.WEST_FLOWER, sim.next());
        sim.applyLeg(TreeState.LegId.WEST_FLOWER);

        assertEquals(TreeState.Selection.NONE, sim.next());
    }

    @Test
    public void cameraSwitchOnPreloadsContinuesFromLoading() {
        Sim sim = new Sim();

        assertEquals(TreeState.Selection.SHOOT_PRELOADS, sim.next());
        // Camera sees no fiducial at the garden shoot: the preload shot
        // happens from the loading side instead.
        sim.applyLeg(TreeState.LegId.SHOOT_PRELOADS, Position.LOADING_SHOOT);

        assertEquals(TreeState.Selection.NORTH_FLOWER, sim.next());
        sim.applyLeg(TreeState.LegId.NORTH_FLOWER);

        // Garden intake (to loading shoot) is next, but it starts at the
        // garden shoot pose: reposition under the hive first.
        assertEquals(TreeState.Selection.REPOSITION, sim.next());
        sim.applyReposition();

        assertEquals(TreeState.Selection.GARDEN_TO_LOADING, sim.next());
        sim.applyLeg(TreeState.LegId.GARDEN_TO_LOADING);

        assertEquals(TreeState.Selection.LOADING_TO_GARDEN, sim.next());
        sim.applyLeg(TreeState.LegId.LOADING_TO_GARDEN);

        assertEquals(TreeState.Selection.WEST_FLOWER, sim.next());
        sim.applyLeg(TreeState.LegId.WEST_FLOWER);

        assertEquals(TreeState.Selection.NONE, sim.next());
    }

    @Test
    public void cameraSwitchOnNorthFlowerContinuesFromGarden() {
        Sim sim = new Sim();
        sim.applyLeg(TreeState.LegId.SHOOT_PRELOADS);
        sim.applyLeg(TreeState.LegId.GARDEN_TO_LOADING);

        assertEquals(TreeState.Selection.NORTH_FLOWER, sim.next());
        // Camera forces the north-flower shot to the garden side.
        sim.applyLeg(TreeState.LegId.NORTH_FLOWER, Position.GARDEN_SHOOT);

        // West flower is available from here and shoots garden (optimal).
        assertEquals(TreeState.Selection.WEST_FLOWER, sim.next());
        sim.applyLeg(TreeState.LegId.WEST_FLOWER);

        // Only the loading preloads remain, starting at the loading side.
        assertEquals(TreeState.Selection.REPOSITION, sim.next());
        sim.applyReposition();

        assertEquals(TreeState.Selection.LOADING_TO_GARDEN, sim.next());
        sim.applyLeg(TreeState.LegId.LOADING_TO_GARDEN);

        assertEquals(TreeState.Selection.NONE, sim.next());
    }

    @Test
    public void repositionNeverPingPongs() {
        // For every reachable-looking state, a REPOSITION selection must
        // be followed by a leg selection on the other side.
        for (Position position : new Position[]{Position.GARDEN_SHOOT, Position.LOADING_SHOOT}) {
            for (int cycles = 0; cycles <= 6; cycles++) {
                for (int mask = 0; mask < 16; mask++) {
                    EnumSet<IntakeArea> done = EnumSet.noneOf(IntakeArea.class);
                    IntakeArea[] areas = IntakeArea.values();
                    for (int i = 0; i < areas.length; i++) {
                        if ((mask & (1 << i)) != 0) done.add(areas[i]);
                    }
                    if (TreeState.selectLeg(position, cycles, done) == TreeState.Selection.REPOSITION) {
                        assertNotEquals("ping-pong at " + position + " cycles=" + cycles + " done=" + done,
                                TreeState.Selection.REPOSITION,
                                TreeState.selectLeg(TreeState.otherShootPosition(position), cycles, done));
                    }
                }
            }
        }
    }

    @Test
    public void noneOnlyWhenAllIntakesDone() {
        // Only the positions the tree can actually be in after the first
        // shot (GARDEN_START only ever occurs with cycles == 0).
        for (Position position : new Position[]{Position.GARDEN_SHOOT, Position.LOADING_SHOOT}) {
            for (int cycles = 1; cycles <= 6; cycles++) {
                for (int mask = 0; mask < 16; mask++) {
                    EnumSet<IntakeArea> done = EnumSet.noneOf(IntakeArea.class);
                    IntakeArea[] areas = IntakeArea.values();
                    for (int i = 0; i < areas.length; i++) {
                        if ((mask & (1 << i)) != 0) done.add(areas[i]);
                    }
                    boolean none = TreeState.selectLeg(position, cycles, done) == TreeState.Selection.NONE;
                    assertEquals("position=" + position + " cycles=" + cycles + " done=" + done,
                            done.size() == IntakeArea.values().length, none);
                }
            }
        }
        // At the start position with no cycles done, the preload leg is
        // always selected.
        assertEquals(TreeState.Selection.SHOOT_PRELOADS,
                TreeState.selectLeg(Position.GARDEN_START, 0, EnumSet.noneOf(IntakeArea.class)));
    }
}
