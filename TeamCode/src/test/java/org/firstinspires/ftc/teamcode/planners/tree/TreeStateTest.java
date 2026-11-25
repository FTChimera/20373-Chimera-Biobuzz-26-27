package org.firstinspires.ftc.teamcode.planners.tree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands.ShotOutcome;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.Test;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

/**
 * Tests for attempt accounting, aborts, the hive tip model, and
 * leg selection. The selection traces simulate the state evolving
 * as legs complete (a leg's shoot may land on the other side when the
 * camera check forces a switch).
 */
public class TreeStateTest {

    private final TreeState state = new TreeState();

    @Test
    public void freshStateDefaultsDoNotInventAttemptsOrConfirmation() {
        assertEquals(Position.GARDEN_START, state.position());
        assertEquals(0, state.attemptedBatches());
        assertEquals(0, state.cyclesCompleted());
        assertEquals(Field.HiveState.UNTIPPED, state.predictedHiveState());
        assertEquals(ShotOutcome.NOT_STARTED, state.lastShotOutcome());
        assertFalse(state.lastShotSettled());
        assertFalse(state.lastShotTagConfirmed());
        assertFalse(state.predictionUncertain());
        assertFalse(state.controlPathAborted());
        assertEquals("", state.controlPathFailure());
        assertTrue(state.intakesDone().isEmpty());
        assertFalse(state.sequenceComplete());
        assertEquals(TreeState.Selection.SHOOT_PRELOADS, state.selectLeg());
    }

    @Test
    public void completedBatchAdvancesAttemptModelButNeverConfirmsScores() {
        state.initialize();
        state.recordAttempt(Position.GARDEN_SHOOT, ShotOutcome.ATTEMPTED, true, true, true);

        assertEquals(1, state.attemptedBatches());
        assertEquals(state.attemptedBatches(), state.cyclesCompleted());
        assertEquals(ShotOutcome.ATTEMPTED, state.lastShotOutcome());
        assertTrue(state.lastShotSettled());
        assertTrue(state.lastShotTagConfirmed());
        // Even readiness plus a fresh tag confirms neither a score nor the tip model.
        assertTrue(state.predictionUncertain());
        assertEquals(Field.HiveState.TIPPED, state.predictedHiveState());
        assertEquals(Position.GARDEN_SHOOT, state.position());
        assertTrue(state.intakesDone().isEmpty());
        assertFalse(state.sequenceComplete());
        assertFalse(state.controlPathAborted());
        assertEquals(TreeState.Selection.GARDEN_TO_LOADING, state.selectLeg());
    }

    @Test
    public void completedBlindAndSpinupTimeoutBatchesStillCountAsAttempts() {
        for (ShotOutcome outcome : new ShotOutcome[]{
                ShotOutcome.ATTEMPTED, ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT}) {
            for (boolean settled : new boolean[]{false, true}) {
                for (boolean tagSeen : new boolean[]{false, true}) {
                    state.initialize();
                    state.recordAttempt(Position.LOADING_SHOOT, outcome, settled, tagSeen, true);

                    assertEquals(1, state.attemptedBatches());
                    assertEquals(1, state.cyclesCompleted());
                    assertEquals(outcome, state.lastShotOutcome());
                    assertEquals(settled, state.lastShotSettled());
                    assertEquals(tagSeen, state.lastShotTagConfirmed());
                    assertTrue(state.predictionUncertain());
                    assertEquals(Field.HiveState.TIPPED, state.predictedHiveState());
                    assertEquals(Position.LOADING_SHOOT, state.position());
                    assertFalse(state.controlPathAborted());
                    assertEquals(TreeState.Selection.NORTH_FLOWER, state.selectLeg());
                }
            }
        }
    }

    @Test
    public void successiveAttemptsRefreshDiagnosticsWithoutRestoringScoreCertainty() {
        state.initialize();
        for (int attempt = 1; attempt <= 5; attempt++) {
            boolean ready = attempt % 2 == 0;
            Position side = ready ? Position.LOADING_SHOOT : Position.GARDEN_SHOOT;
            ShotOutcome outcome = ready
                    ? ShotOutcome.ATTEMPTED : ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT;
            state.recordAttempt(side, outcome, ready, ready, true);

            assertEquals(attempt, state.attemptedBatches());
            assertEquals(attempt, state.cyclesCompleted());
            assertEquals(TreeState.predictHiveState(attempt), state.predictedHiveState());
            assertEquals(outcome, state.lastShotOutcome());
            assertEquals(ready, state.lastShotSettled());
            assertEquals(ready, state.lastShotTagConfirmed());
            assertEquals(side, state.position());
            assertTrue(state.predictionUncertain());
            assertFalse(state.controlPathAborted());
        }
    }

    @Test
    public void incompleteInterruptedAndNullOutcomesCannotMutateProgress() {
        state.initialize();
        state.markIntakeDone(TreeState.IntakeArea.GARDEN);
        state.recordAttempt(Position.GARDEN_SHOOT, ShotOutcome.ATTEMPTED, true, true, true);
        Map<String, Object> before = snapshot();

        for (ShotOutcome outcome : new ShotOutcome[]{
                ShotOutcome.NOT_STARTED, ShotOutcome.WAITING_FOR_SPEED,
                ShotOutcome.FEEDING, ShotOutcome.INTERRUPTED, null}) {
            assertThrows("Rejected outcome: " + outcome, IllegalArgumentException.class,
                    () -> state.recordAttempt(Position.LOADING_SHOOT, outcome, false, false, false));
            assertEquals("Rejected feeds must leave every state field unchanged",
                    before, snapshot());
        }
    }

    @Test
    public void nonShootAndNullPositionsAreRejectedBeforeAnyStateMutation() {
        state.initialize();
        Map<String, Object> before = snapshot();
        for (Position position : new Position[]{Position.GARDEN_START, null}) {
            for (ShotOutcome outcome : new ShotOutcome[]{
                    ShotOutcome.ATTEMPTED, ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT}) {
                assertThrows(IllegalArgumentException.class,
                        () -> state.recordAttempt(position, outcome, true, true, true));
                assertEquals(before, snapshot());
            }
        }
    }

    @Test
    public void unverifiedArrivalCountsAttemptButNeverAdvertisesDestination() {
        state.initialize();
        state.setPosition(Position.GARDEN_SHOOT);
        state.recordAttempt(Position.LOADING_SHOOT, ShotOutcome.ATTEMPTED, true, true, false);

        assertEquals(1, state.attemptedBatches());
        assertEquals(ShotOutcome.ATTEMPTED, state.lastShotOutcome());
        assertTrue(state.predictionUncertain());
        assertTrue(state.lastShotSettled());
        assertTrue(state.lastShotTagConfirmed());
        assertEquals(Position.GARDEN_SHOOT, state.position());
        assertTrue(state.controlPathAborted());
        assertFalse(state.controlPathFailure().isEmpty());
        assertEquals(TreeState.Selection.NONE, state.selectLeg());
        assertFalse(state.sequenceComplete());
    }

    @Test
    public void unverifiedBlindTimeoutAttemptPreservesStartAndAbortsNextLeg() {
        state.initialize();
        state.recordAttempt(Position.GARDEN_SHOOT,
                ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT, false, false, false);

        assertEquals(1, state.attemptedBatches());
        assertEquals(ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT, state.lastShotOutcome());
        assertFalse(state.lastShotSettled());
        assertFalse(state.lastShotTagConfirmed());
        assertTrue(state.predictionUncertain());
        assertEquals(Position.GARDEN_START, state.position());
        assertTrue(state.controlPathAborted());
        assertEquals(TreeState.Selection.NONE, state.selectLeg());
    }

    @Test
    public void skippedShotAdvancesPositionWithoutCountingCycleOrAdvancingTipModel() {
        state.initialize();
        state.setPosition(Position.GARDEN_SHOOT);
        state.recordSkippedShot(Position.GARDEN_SHOOT, true, true, true);

        assertEquals(0, state.attemptedBatches());
        assertEquals(0, state.cyclesCompleted());
        assertEquals(ShotOutcome.SKIPPED, state.lastShotOutcome());
        assertTrue(state.lastShotSettled());
        assertTrue(state.lastShotTagConfirmed());
        // A skip feeds nothing, so the tip model must not advance.
        assertFalse(state.predictionUncertain());
        assertEquals(Field.HiveState.UNTIPPED, state.predictedHiveState());
        assertEquals(Position.GARDEN_SHOOT, state.position());
        assertTrue(state.intakesDone().isEmpty());
        assertFalse(state.controlPathAborted());
        assertFalse(state.sequenceComplete());
    }

    @Test
    public void skippedShotConsumesItsLegSoSelectionMovesOnInsteadOfLooping() {
        state.initialize();
        state.setPosition(Position.GARDEN_SHOOT);
        // The FEED phase's skip handling: record the skip, then mark the
        // leg's intake done. Without the mark, selection would re-pick
        // WEST_FLOWER from the same position with the same cycle count.
        state.recordSkippedShot(Position.GARDEN_SHOOT, true, false, true);
        state.markIntakeDone(TreeState.IntakeArea.WEST_FLOWER);

        assertEquals(0, state.attemptedBatches());
        assertEquals(ShotOutcome.SKIPPED, state.lastShotOutcome());
        assertEquals(TreeState.Selection.GARDEN, state.selectLeg());
    }

    @Test
    public void skippedShotWithoutVerifiedArrivalAbortsWithoutAdvertisingDestination() {
        state.initialize();
        state.setPosition(Position.GARDEN_SHOOT);
        state.recordSkippedShot(Position.LOADING_SHOOT, true, true, false);

        assertEquals(0, state.attemptedBatches());
        assertEquals(ShotOutcome.SKIPPED, state.lastShotOutcome());
        assertEquals(Position.GARDEN_SHOOT, state.position());
        assertTrue(state.controlPathAborted());
        assertFalse(state.controlPathFailure().isEmpty());
        assertEquals(TreeState.Selection.NONE, state.selectLeg());
    }

    @Test
    public void skippedShotRejectsNonShootPositionsBeforeAnyStateMutation() {
        state.initialize();
        Map<String, Object> before = snapshot();
        for (Position position : new Position[]{Position.GARDEN_START, null}) {
            assertThrows(IllegalArgumentException.class,
                    () -> state.recordSkippedShot(position, true, true, true));
            assertEquals(before, snapshot());
        }
    }

    @Test
    public void abortPreventsPreloadWithoutPretendingSequenceCompleted() {
        state.initialize();
        state.abortControlPath("unsafe switch");

        assertEquals(0, state.attemptedBatches());
        assertEquals(ShotOutcome.NOT_STARTED, state.lastShotOutcome());
        assertEquals(Position.GARDEN_START, state.position());
        assertFalse(state.sequenceComplete());
        assertEquals(TreeState.Selection.NONE, state.selectLeg());
    }

    @Test
    public void abortRemainsLatchedAcrossPositionAndIntakeChanges() {
        state.initialize();
        state.recordAttempt(Position.GARDEN_SHOOT, ShotOutcome.ATTEMPTED, true, false, true);
        assertNotEquals(TreeState.Selection.NONE, state.selectLeg());
        state.abortControlPath("interrupted side switch");

        for (Position position : new Position[]{Position.GARDEN_SHOOT, Position.LOADING_SHOOT}) {
            state.setPosition(position);
            state.markIntakeDone(TreeState.IntakeArea.GARDEN);
            assertNotEquals(TreeState.Selection.NONE,
                    TreeState.selectLeg(position, state.cyclesCompleted(), state.intakesDone()));
            assertEquals(TreeState.Selection.NONE, state.selectLeg());
            assertEquals("interrupted side switch", state.controlPathFailure());
        }
    }

    @Test
    public void abortRequiresReasonAndDoesNotEraseExistingFailure() {
        state.initialize();
        state.abortControlPath("verified failure");
        Map<String, Object> before = snapshot();
        for (String reason : new String[]{null, ""}) {
            assertThrows(IllegalArgumentException.class, () -> state.abortControlPath(reason));
            assertEquals(before, snapshot());
        }
    }

    @Test
    public void initializeClearsAbortDiagnosticsProgressAndPreviousIntakeSet() {
        state.initialize();
        state.markIntakeDone(TreeState.IntakeArea.GARDEN);
        EnumSet<TreeState.IntakeArea> oldIntakes = state.intakesDone();
        state.recordAttempt(Position.LOADING_SHOOT,
                ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT, true, true, false);
        state.initialize();

        assertEquals(Position.GARDEN_START, state.position());
        assertEquals(0, state.attemptedBatches());
        assertEquals(0, state.cyclesCompleted());
        assertEquals(Field.HiveState.UNTIPPED, state.predictedHiveState());
        assertEquals(ShotOutcome.NOT_STARTED, state.lastShotOutcome());
        assertFalse(state.lastShotSettled());
        assertFalse(state.lastShotTagConfirmed());
        assertFalse(state.predictionUncertain());
        assertFalse(state.controlPathAborted());
        assertEquals("", state.controlPathFailure());
        assertTrue(state.intakesDone().isEmpty());
        oldIntakes.add(TreeState.IntakeArea.NORTH_FLOWER);
        assertTrue(state.intakesDone().isEmpty());
        assertFalse(state.sequenceComplete());
        assertEquals(TreeState.Selection.SHOOT_PRELOADS, state.selectLeg());
    }

    @Test
    public void sequenceCompletionRequiresAnAttemptAndAllIntakesNotConfirmedScores() {
        state.initialize();
        state.markIntakeDone(null);
        assertTrue(state.intakesDone().isEmpty());
        for (TreeState.IntakeArea area : TreeState.IntakeArea.values()) {
            state.markIntakeDone(area);
            state.markIntakeDone(area);
        }
        assertEquals(EnumSet.allOf(TreeState.IntakeArea.class), state.intakesDone());
        assertFalse(state.sequenceComplete());

        state.recordAttempt(Position.GARDEN_SHOOT,
                ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT, false, false, true);
        assertTrue(state.sequenceComplete());
        assertTrue(state.predictionUncertain());
        assertEquals(1, state.attemptedBatches());
        assertFalse(state.lastShotTagConfirmed());
        assertEquals(TreeState.Selection.NONE, state.selectLeg());
        assertFalse(state.controlPathAborted());
    }

    private Map<String, Object> snapshot() {
        Map<String, Object> values = new HashMap<>();
        values.put("position", state.position());
        values.put("cyclesCompleted", state.cyclesCompleted());
        values.put("predictedHiveState", state.predictedHiveState());
        values.put("intakesDone", EnumSet.copyOf(state.intakesDone()));
        values.put("lastShotOutcome", state.lastShotOutcome());
        values.put("lastShotSettled", state.lastShotSettled());
        values.put("lastShotTagConfirmed", state.lastShotTagConfirmed());
        values.put("predictionUncertain", state.predictionUncertain());
        values.put("controlPathFailure", state.controlPathFailure());
        return values;
    }

    /** Simulates tree state as legs/repositions complete. */
    private static class Sim {
        Position position = Position.GARDEN_START;
        int cycles = 0;
        EnumSet<TreeState.IntakeArea> done = EnumSet.noneOf(TreeState.IntakeArea.class);

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
                    EnumSet<TreeState.IntakeArea> done = EnumSet.noneOf(TreeState.IntakeArea.class);
                    TreeState.IntakeArea[] areas = TreeState.IntakeArea.values();
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
                    EnumSet<TreeState.IntakeArea> done = EnumSet.noneOf(TreeState.IntakeArea.class);
                    TreeState.IntakeArea[] areas = TreeState.IntakeArea.values();
                    for (int i = 0; i < areas.length; i++) {
                        if ((mask & (1 << i)) != 0) done.add(areas[i]);
                    }
                    boolean none = TreeState.selectLeg(position, cycles, done) == TreeState.Selection.NONE;
                    assertEquals("position=" + position + " cycles=" + cycles + " done=" + done,
                            done.size() == TreeState.IntakeArea.values().length, none);
                }
            }
        }
        // At the start position with no cycles done, the preload leg is
        // always selected.
        assertEquals(TreeState.Selection.SHOOT_PRELOADS,
                TreeState.selectLeg(Position.GARDEN_START, 0, EnumSet.noneOf(TreeState.IntakeArea.class)));
    }
}
