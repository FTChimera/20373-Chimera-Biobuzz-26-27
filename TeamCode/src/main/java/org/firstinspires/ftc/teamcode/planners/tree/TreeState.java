package org.firstinspires.ftc.teamcode.planners.tree;

import com.rmsftc.utils.blackboard.Blackboard;

import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands.ShotOutcome;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.game.Field;

import java.util.EnumSet;
import java.util.Set;

/**
 * Typed access to the tree auto's blackboard state, plus the pure logic the
 * tree is built on: the hive tip model and the leg-selection policy.
 *
 * <p>Blackboard keys (enum {@link Key}):
 * <ul>
 *   <li>{@code POSITION} — {@link Position}: last verified route position
 *       (starts GARDEN_START; updated by the shoot step to the side actually
 *       shot from, and by the reposition command).</li>
 *   <li>{@code CYCLES_COMPLETED} — Integer: timed batch attempts completed,
 *       not confirmed scores.</li>
 *   <li>{@code PREDICTED_HIVE_STATE} — {@link Field.HiveState} from the tip
 *       model (stored so telemetry can show it; recomputed on every shot).</li>
 *   <li>{@code INTAKES_DONE} — {@code EnumSet<IntakeArea>}: intake areas
 *       already collected from.</li>
 * </ul>
 *
 * <p>Tip model (an unverified all-shots-score hypothesis): the preload attempt is cycle 1. The
 * hive tips after cycles 1, 3, 5, ... (first cycle, then every 2), and tips
 * alternate direction starting from UNTIPPED (garden side). So
 * tips = (cyclesCompleted + 1) / 2 and the predicted state is TIPPED when
 * tips is odd, UNTIPPED when even. The optimal side for the next shoot is
 * the side matching the predicted state.
 *
 * <p>Selection policy ({@link #selectLeg(Position, int, Set)}), in order:
 * <ol>
 *   <li>A leg from the current position whose intake is still available and
 *       whose shoot side is the optimal side for the next cycle.</li>
 *   <li>REPOSITION, if such a leg exists from the other shoot position.</li>
 *   <li>Any leg from the current position with an available intake (the
 *       camera check at shoot time will switch sides if needed).</li>
 *   <li>REPOSITION, if any leg with an available intake exists from the
 *       other shoot position.</li>
 *   <li>NONE — only when every intake is done (the park branch wins then).</li>
 * </ol>
 * REPOSITION is only returned when a leg exists on the other side, so the
 * robot can never ping-pong. Under perfect prediction this reproduces
 * GardenSolo's order exactly: ShootPreloads, ToIntakeGardenToLoading,
 * ToIntakeFromNorthFlower, ToIntakeLoadingToGarden, ToIntakeFromWestFlower,
 * then NONE (park).
 */
public final class TreeState {

    /** Blackboard keys for the tree state. */
    public enum Key {
        POSITION,
        CYCLES_COMPLETED,
        PREDICTED_HIVE_STATE,
        INTAKES_DONE,
        LAST_SHOT_OUTCOME,
        LAST_SHOT_SETTLED,
        LAST_SHOT_TAG_CONFIRMED,
        PREDICTION_UNCERTAIN,
        CONTROL_PATH_FAILURE
    }

    /**
     * The legs the tree can pick, in selection priority order (GardenSolo's
     * order first, then the alternates). Each leg knows where it starts
     * ({@link #from()}), where it ends/shoots ({@link #to()}), and which
     * intake area it collects ({@link #intakeArea()}, null for the preload
     * leg). REPOSITION and NONE are selection outcomes, not legs.
     */
    public enum LegId {
        SHOOT_PRELOADS(Position.GARDEN_START, Position.GARDEN_SHOOT, null),
        GARDEN_TO_LOADING(Position.GARDEN_SHOOT, Position.LOADING_SHOOT, IntakeArea.GARDEN),
        NORTH_FLOWER(Position.LOADING_SHOOT, Position.LOADING_SHOOT, IntakeArea.NORTH_FLOWER),
        LOADING_TO_GARDEN(Position.LOADING_SHOOT, Position.GARDEN_SHOOT, IntakeArea.LOADING_PRELOADS),
        WEST_FLOWER(Position.GARDEN_SHOOT, Position.GARDEN_SHOOT, IntakeArea.WEST_FLOWER),
        GARDEN(Position.GARDEN_SHOOT, Position.GARDEN_SHOOT, IntakeArea.GARDEN),
        WEST_FLOWER_TO_GARDEN(Position.LOADING_SHOOT, Position.GARDEN_SHOOT, IntakeArea.WEST_FLOWER);

        private final Position from;
        private final Position to;
        private final IntakeArea intakeArea;

        LegId(Position from, Position to, IntakeArea intakeArea) {
            this.from = from;
            this.to = to;
            this.intakeArea = intakeArea;
        }

        public Position from() {
            return from;
        }

        /** The shoot position this leg shoots from (its end position). */
        public Position to() {
            return to;
        }

        /** The intake area this leg collects, or null for the preload leg. */
        public IntakeArea intakeArea() {
            return intakeArea;
        }
    }

    /** Outcome of {@link #selectLeg(Position, int, Set)}. */
    public enum Selection {
        SHOOT_PRELOADS,
        GARDEN_TO_LOADING,
        NORTH_FLOWER,
        LOADING_TO_GARDEN,
        WEST_FLOWER,
        GARDEN,
        WEST_FLOWER_TO_GARDEN,
        /** Drive the under-hive path to the other shoot position. */
        REPOSITION,
        /** Nothing left to do (park branch takes over). */
        NONE;

        /** The leg this selection names, or null for REPOSITION/NONE. */
        public LegId leg() {
            switch (this) {
                case SHOOT_PRELOADS: return LegId.SHOOT_PRELOADS;
                case GARDEN_TO_LOADING: return LegId.GARDEN_TO_LOADING;
                case NORTH_FLOWER: return LegId.NORTH_FLOWER;
                case LOADING_TO_GARDEN: return LegId.LOADING_TO_GARDEN;
                case WEST_FLOWER: return LegId.WEST_FLOWER;
                case GARDEN: return LegId.GARDEN;
                case WEST_FLOWER_TO_GARDEN: return LegId.WEST_FLOWER_TO_GARDEN;
                default: return null;
            }
        }

        public static Selection of(LegId leg) {
            switch (leg) {
                case SHOOT_PRELOADS: return SHOOT_PRELOADS;
                case GARDEN_TO_LOADING: return GARDEN_TO_LOADING;
                case NORTH_FLOWER: return NORTH_FLOWER;
                case LOADING_TO_GARDEN: return LOADING_TO_GARDEN;
                case WEST_FLOWER: return WEST_FLOWER;
                case GARDEN: return GARDEN;
                case WEST_FLOWER_TO_GARDEN: return WEST_FLOWER_TO_GARDEN;
                default: throw new IllegalArgumentException("Unknown leg: " + leg);
            }
        }
    }

    private final Blackboard board;

    public TreeState(Blackboard board) {
        this.board = board;
    }

    /** Resets all tree state to the start-of-auto values. */
    public void initialize() {
        board.set(Key.POSITION, Position.GARDEN_START);
        board.set(Key.CYCLES_COMPLETED, 0);
        board.set(Key.PREDICTED_HIVE_STATE, Field.HiveState.UNTIPPED);
        board.set(Key.INTAKES_DONE, EnumSet.noneOf(IntakeArea.class));
        board.set(Key.LAST_SHOT_OUTCOME, ShotOutcome.NOT_STARTED);
        board.set(Key.LAST_SHOT_SETTLED, false);
        board.set(Key.LAST_SHOT_TAG_CONFIRMED, false);
        board.set(Key.PREDICTION_UNCERTAIN, false);
        board.set(Key.CONTROL_PATH_FAILURE, "");
    }

    public Position position() {
        return board.getOrDefault(Key.POSITION, Position.class, Position.GARDEN_START);
    }

    public void setPosition(Position position) {
        board.set(Key.POSITION, position);
    }

    /** Compatibility name: these are completed timed attempts, not scored batches. */
    public int cyclesCompleted() {
        return attemptedBatches();
    }

    public int attemptedBatches() {
        return board.getOrDefault(Key.CYCLES_COMPLETED, Integer.class, 0);
    }

    public ShotOutcome lastShotOutcome() {
        return board.getOrDefault(Key.LAST_SHOT_OUTCOME, ShotOutcome.class, ShotOutcome.NOT_STARTED);
    }

    public boolean lastShotSettled() {
        return board.getOrDefault(Key.LAST_SHOT_SETTLED, Boolean.class, false);
    }

    public boolean lastShotTagConfirmed() {
        return board.getOrDefault(Key.LAST_SHOT_TAG_CONFIRMED, Boolean.class, false);
    }

    public boolean predictionUncertain() {
        return board.getOrDefault(Key.PREDICTION_UNCERTAIN, Boolean.class, false);
    }

    public String controlPathFailure() {
        return board.getOrDefault(Key.CONTROL_PATH_FAILURE, String.class, "");
    }

    public boolean controlPathAborted() {
        return !controlPathFailure().isEmpty();
    }

    /** Latches failure until initialize; never launch another leg from an invented pose. */
    public void abortControlPath(String reason) {
        if (reason == null || reason.isEmpty()) {
            throw new IllegalArgumentException("A control-path failure reason is required");
        }
        board.set(Key.CONTROL_PATH_FAILURE, reason);
    }

    public Field.HiveState predictedHiveState() {
        return board.getOrDefault(Key.PREDICTED_HIVE_STATE, Field.HiveState.class, Field.HiveState.UNTIPPED);
    }

    @SuppressWarnings("unchecked")
    public EnumSet<IntakeArea> intakesDone() {
        return board.getOrDefault(Key.INTAKES_DONE, EnumSet.class, EnumSet.noneOf(IntakeArea.class));
    }

    public void markIntakeDone(IntakeArea area) {
        if (area == null) return;
        EnumSet<IntakeArea> done = intakesDone();
        done.add(area);
        board.set(Key.INTAKES_DONE, done);
    }

    /** True once a batch was attempted and all four intakes were visited. */
    public boolean sequenceComplete() {
        return cyclesCompleted() > 0 && intakesDone().size() == IntakeArea.values().length;
    }

    /**
     * Advances sequence progress only for a completed timed batch attempt.
     * Neither speed readiness nor tag visibility confirms scored pollen;
     * the tip prediction becomes uncertain even for a non-degraded attempt.
     * Position changes only when measured arrival was verified.
     */
    public void recordAttempt(Position shotFrom, ShotOutcome outcome, boolean settled,
                              boolean freshTag, boolean positionVerified) {
        if (outcome != ShotOutcome.ATTEMPTED && outcome != ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT) {
            throw new IllegalArgumentException("Only a completed feed counts as a batch attempt");
        }
        otherShootPosition(shotFrom);
        int cycles = attemptedBatches() + 1;
        board.set(Key.CYCLES_COMPLETED, cycles);
        board.set(Key.PREDICTED_HIVE_STATE, predictHiveState(cycles));
        board.set(Key.PREDICTION_UNCERTAIN, true);
        board.set(Key.LAST_SHOT_OUTCOME, outcome);
        board.set(Key.LAST_SHOT_SETTLED, settled);
        board.set(Key.LAST_SHOT_TAG_CONFIRMED, freshTag);
        if (positionVerified) {
            board.set(Key.POSITION, shotFrom);
        } else {
            abortControlPath("Batch attempted without verified arrival at " + shotFrom);
        }
    }

    /** The leg (or reposition/none) the tree should run next. */
    public Selection selectLeg() {
        if (controlPathAborted()) return Selection.NONE;
        return selectLeg(position(), cyclesCompleted(), intakesDone());
    }

    // ---- Pure logic (unit-tested) ----

    /**
     * Predicted hive state after {@code cyclesCompleted} shoots, assuming
     * every shot scores: tips = (cycles + 1) / 2; odd tips = TIPPED
     * (loading side), even = UNTIPPED (garden side).
     */
    public static Field.HiveState predictHiveState(int cyclesCompleted) {
        int tips = (cyclesCompleted + 1) / 2;
        return (tips % 2 == 1) ? Field.HiveState.TIPPED : Field.HiveState.UNTIPPED;
    }

    /** The shoot position matching the predicted state for the next shoot. */
    public static Position optimalShootPosition(int cyclesCompleted) {
        return predictHiveState(cyclesCompleted) == Field.HiveState.UNTIPPED
                ? Position.GARDEN_SHOOT
                : Position.LOADING_SHOOT;
    }

    /** The other shoot position (GARDEN_SHOOT &lt;-&gt; LOADING_SHOOT). */
    public static Position otherShootPosition(Position position) {
        if (position == Position.GARDEN_SHOOT) return Position.LOADING_SHOOT;
        if (position == Position.LOADING_SHOOT) return Position.GARDEN_SHOOT;
        throw new IllegalArgumentException("Not a shoot position: " + position);
    }

    /** The field pose for a {@link Position} (used as a leg's start pose). */
    public static Pose poseFor(Position position) {
        switch (position) {
            case GARDEN_START: return StartPoses.gardenArea;
            case GARDEN_SHOOT: return ShootPoses.garden;
            case LOADING_SHOOT: return ShootPoses.loading;
            default: throw new IllegalArgumentException("Unknown position: " + position);
        }
    }

    private static boolean intakeAvailable(LegId leg, int cyclesCompleted, Set<IntakeArea> done) {
        return leg.intakeArea() == null
                ? cyclesCompleted == 0
                : !done.contains(leg.intakeArea());
    }

    /** The selection policy described in the class javadoc. Pure. */
    public static Selection selectLeg(Position position, int cyclesCompleted, Set<IntakeArea> done) {
        Position optimal = optimalShootPosition(cyclesCompleted);
        // 1. Filtered leg from here.
        for (LegId leg : LegId.values()) {
            if (leg.from() == position && leg.to() == optimal && intakeAvailable(leg, cyclesCompleted, done)) {
                return Selection.of(leg);
            }
        }
        // 2. Filtered leg from the other shoot position -> reposition.
        if (position == Position.GARDEN_SHOOT || position == Position.LOADING_SHOOT) {
            Position other = otherShootPosition(position);
            for (LegId leg : LegId.values()) {
                if (leg.from() == other && leg.to() == optimal && intakeAvailable(leg, cyclesCompleted, done)) {
                    return Selection.REPOSITION;
                }
            }
        }
        // 3. Unfiltered leg from here (camera decides at shoot time).
        for (LegId leg : LegId.values()) {
            if (leg.from() == position && intakeAvailable(leg, cyclesCompleted, done)) {
                return Selection.of(leg);
            }
        }
        // 4. Unfiltered leg from the other shoot position -> reposition.
        if (position == Position.GARDEN_SHOOT || position == Position.LOADING_SHOOT) {
            Position other = otherShootPosition(position);
            for (LegId leg : LegId.values()) {
                if (leg.from() == other && intakeAvailable(leg, cyclesCompleted, done)) {
                    return Selection.REPOSITION;
                }
            }
        }
        return Selection.NONE;
    }
}
