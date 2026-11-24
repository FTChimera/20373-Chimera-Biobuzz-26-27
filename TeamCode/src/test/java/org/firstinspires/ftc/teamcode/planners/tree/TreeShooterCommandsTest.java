package org.firstinspires.ftc.teamcode.planners.tree;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import com.rmsftc.utils.math.Pose;
import com.rmsftc.cmdlib.Status;
import org.firstinspires.ftc.teamcode.pedro.FollowerTestRig;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.tree.TreeShooterCommands.VisionAction;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.util.FakeClock;

public class TreeShooterCommandsTest {
    private static final long MS = 1_000_000L;

    @Test
    public void unmetSwitchPrerequisiteKeepsBlindFallbackRatherThanStartingUnsafeSwitch() {
        assertTrue(TreeShooterCommands.canSwitchSides(true, false, 1));
        assertFalse(TreeShooterCommands.canSwitchSides(true, false, 1.5));
        assertFalse(TreeShooterCommands.canSwitchSides(false, false, 0));
        assertFalse(TreeShooterCommands.canSwitchSides(true, true, 0));
        assertFalse(TreeShooterCommands.canSwitchSides(true, false, Double.NaN));
    }

    @Test
    public void freshTagAgreeingWithIntendedSideFeedsWithoutSwitch() {
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.UNTIPPED, Position.GARDEN_SHOOT, true, false, 0));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.LOADING_SHOOT, true, false, 0));
    }

    @Test
    public void freshTagDisagreeingWithIntendedSideSwitchesOnceWhenSettled() {
        assertEquals(VisionAction.SWITCH, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, true, false, 0));
        assertEquals(VisionAction.SWITCH, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.UNTIPPED, Position.LOADING_SHOOT, true, false, 1));
    }

    @Test
    public void noFreshTagNeverSwitches() {
        // The reported bug: tag absence used to switch sides. Absence is
        // also the camera-failure signature, so a blind camera feeds from
        // the intended side — even when settled in position, and even if a
        // (stale) inference happens to disagree.
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                false, null, Position.GARDEN_SHOOT, true, false, 0));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                false, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, true, false, 0));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                false, null, Position.LOADING_SHOOT, true, false, 0));
    }

    @Test
    public void ambiguousFreshFrameNeverSwitches() {
        // A fresh frame whose clusters are too close to call yields a null
        // inference: no evidence, so no switch even when settled.
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, null, Position.GARDEN_SHOOT, true, false, 0));
    }

    @Test
    public void disagreeingEvidenceWithoutSwitchPrerequisitesFeeds() {
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, false, false, 0));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, true, true, 0));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, true, false, 1.5));
        assertEquals(VisionAction.FEED, TreeShooterCommands.decideVisionAction(
                true, Field.HiveState.TIPPED, Position.GARDEN_SHOOT, true, false, Double.NaN));
    }

    @Test
    public void distantApproachDoesNotBecomeStraightLineDriveToShootingPose() {
        Pose actual = new Pose(20, 30, 0);
        Pose target = new Pose(60, 120, 1);
        Pose hold = TreeShooterCommands.shootingHoldPose(actual, target);
        assertEquals(20, hold.x(), 0);
        assertEquals(30, hold.y(), 0);
        assertEquals(1, hold.heading(), 0);
        assertEquals(0, actual.distance(hold), 0);
    }

    @Test
    public void nearbyApproachUsesPlainPedroTargetNotHiddenBaseFields() {
        Pose actual = new Pose(60.5, 120, 0);
        Pose hold = TreeShooterCommands.shootingHoldPose(actual, new Pose(60, 120, 1));
        assertEquals(60, hold.x(), 0);
        assertEquals(120, hold.y(), 0);
        assertEquals(1, hold.heading(), 0);
        assertEquals(0.5, actual.distance(hold), 0);
    }

    private static final class SwitchRig {
        final FollowerTestRig drive = new FollowerTestRig();
        final FakeClock now = new FakeClock();
        final List<String> warnings = new ArrayList<>();
        final TreeShooterCommands.SideSwitchCommand command;

        SwitchRig(Position from) {
            drive.pose = Pose.fromPedro(TreeState.poseFor(from));
            command = new TreeShooterCommands.SideSwitchCommand(
                    drive.follower, from, warnings::add, now);
        }

        void settleCurrentHold() {
            drive.pose = drive.follower.holdPose;
            drive.follower.update(0.02);
            assertEquals(Status.RUNNING, command.execute());
            now.add(150 * MS);
            drive.follower.update(0.02);
            command.execute();
        }

        void enterCorridor() {
            assertEquals(Status.RUNNING, command.execute());
            settleCurrentHold();
            assertTrue(drive.follower.following());
        }
    }

    @Test
    public void sideSwitchRejectsUnverifiedOrNonfiniteStartWithoutClaimingArrival() {
        for (double offset : new double[]{1.5, Double.NaN}) {
            SwitchRig rig = new SwitchRig(Position.GARDEN_SHOOT);
            Pose start = Pose.fromPedro(rig.drive.pose);
            rig.drive.pose = new Pose(start.x() + offset, start.y(), start.heading());
            rig.command.execute();
            assertEquals(TreeShooterCommands.SwitchOutcome.ABORTED, rig.command.outcome());
            assertFalse(rig.command.reachedDestination());
            assertTrue(rig.drive.follower.idle());
            assertFalse(rig.warnings.isEmpty());
        }
    }

    @Test
    public void sideSwitchTurnCannotFinishFromBusyFlagOrOldCompletion() {
        SwitchRig rig = new SwitchRig(Position.GARDEN_SHOOT);
        rig.command.execute();
        rig.drive.busy = false;
        rig.drive.atEnd = true;
        rig.drive.progress = 1;
        rig.now.set(Timings.flowerTurnTimeoutMs * MS);
        rig.command.execute();
        assertEquals(TreeShooterCommands.SwitchOutcome.ABORTED, rig.command.outcome());
        assertTrue(rig.drive.follower.idle());
        assertFalse(rig.command.reachedDestination());
    }

    @Test
    public void sideSwitchStalledCorridorIsBoundedAndStopsControl() {
        SwitchRig rig = new SwitchRig(Position.GARDEN_SHOOT);
        rig.enterCorridor();
        rig.now.add(6000 * MS);
        rig.command.execute();
        assertEquals(TreeShooterCommands.SwitchOutcome.ABORTED, rig.command.outcome());
        assertTrue(rig.drive.follower.idle());
        assertTrue(rig.command.failureReason().contains("traversal timed out"));
        assertFalse(rig.command.reachedDestination());
    }

    @Test
    public void parametricEndFarFromDestinationAbortsInsteadOfCuttingAcrossField() {
        SwitchRig rig = new SwitchRig(Position.GARDEN_SHOOT);
        rig.enterCorridor();
        rig.drive.atEnd = true;
        rig.drive.progress = 1;
        rig.drive.follower.update(0.02);
        rig.command.execute();
        assertEquals(TreeShooterCommands.SwitchOutcome.ABORTED, rig.command.outcome());
        assertFalse(rig.command.reachedDestination());
        assertTrue(rig.drive.follower.idle());
        assertTrue(rig.command.failureReason().contains("without measured destination"));
    }

    @Test
    public void replacedCorridorControlAbortsInsteadOfResumingOrShooting() {
        SwitchRig rig = new SwitchRig(Position.GARDEN_SHOOT);
        rig.enterCorridor();
        rig.drive.follower.manual(0, 0, 0);
        rig.command.execute();
        assertEquals(TreeShooterCommands.SwitchOutcome.ABORTED, rig.command.outcome());
        assertTrue(rig.drive.follower.idle());
        assertFalse(rig.command.reachedDestination());
    }

    @Test
    public void interruptedSwitchStopsWithoutSuccessfulDestinationOutcome() {
        SwitchRig rig = new SwitchRig(Position.LOADING_SHOOT);
        rig.enterCorridor();
        rig.command.invalidate();
        assertEquals(TreeShooterCommands.SwitchOutcome.INTERRUPTED, rig.command.outcome());
        assertFalse(rig.command.reachedDestination());
        assertTrue(rig.drive.follower.idle());
    }

    @Test
    public void bothSwitchDirectionsRequireArrivalAndFinalHeadingDwell() {
        for (Position from : new Position[]{Position.GARDEN_SHOOT, Position.LOADING_SHOOT}) {
            SwitchRig rig = new SwitchRig(from);
            rig.enterCorridor();
            rig.drive.pose = rig.drive.follower.currentPath().endPose();
            rig.drive.progress = 1;
            rig.drive.atEnd = true;
            rig.drive.follower.update(0.02);
            rig.command.execute();
            assertEquals(rig.command.failureReason() + " actual=" + rig.drive.pose
                            + " target=" + TreeState.poseFor(TreeState.otherShootPosition(from)),
                    TreeShooterCommands.SwitchOutcome.RUNNING,
                    rig.command.outcome());
            assertFalse(rig.command.reachedDestination());
            rig.settleCurrentHold();
            assertFalse(rig.command.reachedDestination());
            rig.settleCurrentHold();
            assertEquals(TreeShooterCommands.SwitchOutcome.REACHED, rig.command.outcome());
            assertTrue(rig.command.reachedDestination());
            assertTrue(rig.warnings.isEmpty());
        }
    }

    @Test
    public void readinessRequiresContinuous150MillisecondDwell() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        assertFalse(window.settled(true, 0));
        assertFalse(window.settled(true, 149 * MS));
        assertFalse(window.settled(true, 150 * MS - 1));
        assertTrue(window.settled(true, 150 * MS));
        assertTrue(window.settled(true, 151 * MS));
    }

    @Test
    public void dwellStartsAtFirstReadySampleNotWindowCreation() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        assertFalse(window.settled(false, 0));
        assertFalse(window.settled(false, 500 * MS));
        assertFalse(window.settled(true, 1000 * MS));
        assertFalse(window.settled(true, 1150 * MS - 1));
        assertTrue(window.settled(true, 1150 * MS));
    }

    @Test
    public void badSampleDiscardsEntirePreviousDwell() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        assertFalse(window.settled(true, 0));
        assertFalse(window.settled(true, 149 * MS));
        assertFalse(window.settled(false, 150 * MS));
        assertFalse(window.settled(false, 300 * MS));
        assertFalse(window.settled(true, 400 * MS));
        assertFalse(window.settled(true, 550 * MS - 1));
        assertTrue(window.settled(true, 550 * MS));
    }

    @Test
    public void badSampleAlsoRevokesPreviouslySettledState() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        assertFalse(window.settled(true, 0));
        assertTrue(window.settled(true, 150 * MS));
        assertFalse(window.settled(false, 151 * MS));
        assertFalse(window.settled(true, 152 * MS));
        assertFalse(window.settled(true, 302 * MS - 1));
        assertTrue(window.settled(true, 302 * MS));
    }

    @Test
    public void repeatedSamplesAtSameTimeCannotAccumulateDwell() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        for (int sample = 0; sample < 200; sample++) {
            assertFalse(window.settled(true, 100 * MS));
        }
        assertFalse(window.settled(true, 250 * MS - 1));
        assertTrue(window.settled(true, 250 * MS));
    }

    @Test
    public void timeoutIsBoundedAtExactDeadlineEvenWithoutReadiness() {
        long start = 25 * MS;
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(start, 1500);

        assertFalse(window.timedOut(start));
        assertFalse(window.settled(false, start + 1499 * MS));
        assertFalse(window.timedOut(start + 1500 * MS - 1));
        assertTrue(window.timedOut(start + 1500 * MS));
        assertFalse(window.settled(false, start + 1500 * MS));
        assertTrue(window.timedOut(start + 2000 * MS));
    }

    @Test
    public void readinessFlappingNeverExtendsOriginalTimeout() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500);

        for (int millis = 0; millis < 1500; millis += 50) {
            assertFalse(window.settled(millis % 100 == 0, millis * MS));
            assertFalse(window.timedOut(millis * MS));
        }
        assertFalse(window.settled(true, 1500 * MS));
        assertTrue(window.timedOut(1500 * MS));
    }

    @Test
    public void timeoutPreservesFractionalMilliseconds() {
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(0, 1500.5);

        assertFalse(window.timedOut(1500 * MS));
        assertFalse(window.timedOut(1500 * MS + MS / 2 - 1));
        assertTrue(window.timedOut(1500 * MS + MS / 2));
    }

    @Test
    public void monotonicNanosecondWrapPreservesDwellAndTimeout() {
        long start = Long.MAX_VALUE - 75 * MS;
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(start, 1500);

        assertFalse(window.settled(true, start));
        assertFalse(window.settled(true, start + 150 * MS - 1));
        assertTrue(window.settled(true, start + 150 * MS));
        assertFalse(window.timedOut(start + 1500 * MS - 1));
        assertTrue(window.timedOut(start + 1500 * MS));
    }

    @Test
    public void badSampleAcrossMonotonicWrapStillRestartsDwell() {
        long start = Long.MAX_VALUE - 75 * MS;
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(start, 1500);

        assertFalse(window.settled(true, start));
        assertFalse(window.settled(false, start + 100 * MS));
        assertFalse(window.settled(true, start + 110 * MS));
        assertFalse(window.settled(true, start + 260 * MS - 1));
        assertTrue(window.settled(true, start + 260 * MS));
    }

    @Test
    public void negativeNanosecondOriginIsNotAnUnstartedSentinel() {
        long start = -1000 * MS;
        TreeShooterCommands.SettleWindow window = new TreeShooterCommands.SettleWindow(start, 1500);

        assertFalse(window.settled(true, start));
        assertFalse(window.settled(true, start + 150 * MS - 1));
        assertTrue(window.settled(true, start + 150 * MS));
        assertFalse(window.timedOut(start + 1500 * MS - 1));
        assertTrue(window.timedOut(start + 1500 * MS));
    }
}
