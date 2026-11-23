package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.bttree.BehaviorNode;
import com.rmsftc.bttree.nodes.ConditionNode;
import com.rmsftc.bttree.nodes.ReactiveSelectorNode;
import com.rmsftc.bttree.nodes.SequenceNode;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.ftc.ArchitectureConverter;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.utils.blackboard.Blackboard;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.modules.TreeLegs;
import org.firstinspires.ftc.teamcode.planners.tree.ParkNode;
import org.firstinspires.ftc.teamcode.planners.tree.TreeShooterCommands;
import org.firstinspires.ftc.teamcode.planners.tree.TreeState;
import com.rmsftc.utils.RobotMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Garden solo auto, behavior-tree version: garden start, shoot preloads,
 * then repeatedly pick the next intake leg from the blackboard state
 * (position, cycles completed, predicted hive state, intakes done) and
 * park when time runs low or everything is done.
 *
 * <p>Tree shape: a reactive selector root with two branches. The park
 * branch (highest priority) fires when {@code robot.shouldPark()} or the
 * sequence is complete, preempting whatever leg is running. The main
 * branch is a reactive selector with one branch per leg plus a
 * reposition branch; each leg branch is a sequence of [selection
 * condition, leg command as a node]. Selection and the hive tip model
 * live in {@link TreeState}: under perfect prediction (every shot tips
 * on schedule) the tree reproduces the old fixed order — ShootPreloads,
 * ToIntakeGardenToLoading, ToIntakeFromNorthFlower,
 * ToIntakeLoadingToGarden, ToIntakeFromWestFlower, park. Each leg's
 * shoot step checks the camera once and switches sides under the hive
 * if no alliance fiducial is visible (see {@link TreeShooterCommands});
 * the blackboard position then reflects the side actually shot from, so
 * selection continues from reality.
 *
 * <p>Node semantics that matter here: {@code SequenceNode} has memory —
 * once a leg branch passes its condition it stays on the leg node until
 * the leg finishes, so a running leg is never restarted by re-ticking.
 * {@code ReactiveSelectorNode} re-evaluates from the top every tick, so
 * the park branch preempts mid-leg, and finished leg branches re-check
 * their conditions against the updated blackboard.
 */

@Autonomous(name = "Garden Solo Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class GardenSoloAuto extends ArchitecturalOpMode {
    Robot robot;
    boolean isStopped = false;
    private boolean started;
    private TreeState state;

    @Override
    protected void onInit() {
        OpModeStorage.pose.set(StartPoses.gardenArea);
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setSmartShooterControl(true);
    }

    @Override
    protected void build(ArchitectureManager manager) {
        Blackboard board = manager.blackboard();
        state = new TreeState(board);
        state.initialize();

        // Park branch: highest priority. The started guard keeps the
        // condition safe on the very first tick, which runs before
        // onLoop() calls robot.start() (shouldPark reads the match timer).
        BehaviorNode parkBranch = new SequenceNode("Park", board, Arrays.asList(
                new ConditionNode("ShouldPark", board,
                        () -> started && (robot.shouldPark() || state.sequenceComplete())),
                new ParkNode("Park", board, robot)));

        List<BehaviorNode> legBranches = new ArrayList<>();
        for (TreeState.LegId legId : TreeState.LegId.values()) {
            legBranches.add(legBranch(legId, board));
        }
        legBranches.add(new SequenceNode("Reposition", board, Arrays.asList(
                new ConditionNode("Select:REPOSITION", board,
                        () -> state.selectLeg() == TreeState.Selection.REPOSITION),
                ArchitectureConverter.toNode(TreeShooterCommands.reposition(robot, state), board, "Reposition"))));
        BehaviorNode legsSelector = new ReactiveSelectorNode("Legs", board, legBranches);

        manager.jTree(b -> b.node(parkBranch).node(legsSelector));
    }

    private BehaviorNode legBranch(TreeState.LegId legId, Blackboard board) {
        PathPlanner leg = TreeLegs.create(legId, robot, state);
        // The leg's start pose comes from the blackboard position (the
        // selection condition guarantees it matches leg.from()); the
        // intake is marked done when the leg completes.
        Command legCmd = Commands.sequence(
                Commands.runOnce(() -> leg.setStartPose(TreeState.poseFor(state.position()))),
                leg,
                Commands.runOnce(() -> state.markIntakeDone(legId.intakeArea())));
        return new SequenceNode("Leg:" + legId.name(), board, Arrays.asList(
                new ConditionNode("Select:" + legId.name(), board,
                        () -> state.selectLeg() == TreeState.Selection.of(legId)),
                ArchitectureConverter.toNode(legCmd, board, legId.name())));
    }

    @Override
    protected void onLoop() {
        // ArchitecturalOpMode has no start() hook: robot.start() (match
        // timer, lights, vision) runs on the first loop tick, before the
        // first robot.update() that reads the timer.
        if (!started) {
            robot.start();
            started = true;
        }
        if (robot.opModeTimeRemaining() <= 0) {
            stopAutonomous();
            return;
        }

        robot.update();
        telemetry.addData("GardenSolo tree position", state.position());
        telemetry.addData("GardenSolo tree predicted hive", state.predictedHiveState());
        telemetry.addData("GardenSolo tree cycles", state.cyclesCompleted());
        telemetry.addData("GardenSolo tree intakes done", state.intakesDone());
        telemetry.addData("GardenSolo tree next", state.selectLeg());
        telemetry.addData("GardenSolo follower mode", robot.follower.mode());
        telemetry.addData("GardenSolo follower path", robot.follower.pathIndex());
    }

    @Override
    public void onStop() {
        if (!isStopped) {
            OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
            robot.stopEverything();
            isStopped = true;
        }
        super.onStop();
    }

    private void stopAutonomous() {
        if (isStopped) {
            return;
        }
        OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
        robot.stopEverything();
        isStopped = true;
        interrupt();
    }
}
