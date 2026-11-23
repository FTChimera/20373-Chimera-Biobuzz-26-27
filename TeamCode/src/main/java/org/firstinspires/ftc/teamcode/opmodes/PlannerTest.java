package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.modules.FromShootGardenPose;
import org.firstinspires.ftc.teamcode.planners.modules.FromShootLoadingPose;
import org.firstinspires.ftc.teamcode.planners.modules.FromStartGardenPose;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Planner Test: steps through every PathPlanner the autos use, one at a
 * time, so each path (and its heading) can be watched on the field.
 *
 * There is NO 30-second time limit: the OpMode runs until the driver
 * stops it. Place the robot at the garden start pose before pressing
 * start, then press gamepad1 A to run the next planner in the tour.
 * Each step runs the planner (a BA Command) through the manager's
 * scheduler, exactly as the autos do -- drive, deployer, dwell,
 * lineup and shoot included.
 *
 * Tour (garden start; one garden-to-loading leg runs twice, because
 * three loading-to-garden planners each need their own arrival at
 * the loading shoot pose):
 *   1 Shoot Preloads at Garden
 *   2 Intake Garden
 *   3 Intake West Flower
 *   4 Intake North Flower to Loading
 *   5 Intake North Flower
 *   6 Intake West Flower to Garden
 *   7 Intake Garden to Loading
 *   8 Intake Loading to Garden
 *   9 Intake Garden to Loading (again)
 *  10 Intake Partner Preloads to Garden
 *
 * When the tour is done, stop the OpMode, re-place the robot at the
 * garden start pose, and re-init to run it again. There is no restart
 * button: re-running from the middle would start a planner from the
 * wrong pose.
 */
@Autonomous(name = "Planner Test", group = "test", preselectTeleOp = MainTeleOp.NAME)
public class PlannerTest extends ArchitecturalOpMode {
    Robot robot;
    boolean isStopped = false;
    Command currentPlan;
    PathPlanner currentPlanner;
    Pose currentPose;
    List<Step> tour;
    int tourIndex;
    boolean running;
    boolean prevA;
    private boolean started;

    private static class Step {
        final PathPlanner planner;
        final String name;

        Step(PathPlanner planner, String name) {
            this.planner = planner;
            this.name = name;
        }
    }

    private void addStep(PathPlanner planner) {
        tour.add(new Step(planner, planner.toString()));
    }

    private void addStep(PathPlanner planner, String suffix) {
        tour.add(new Step(planner, planner.toString() + suffix));
    }

    @Override
    protected void onInit() {
        OpModeStorage.pose.set(StartPoses.gardenArea);
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setSmartShooterControl(true);
        currentPose = StartPoses.gardenArea;
        tour = new ArrayList<>();
        addStep(new FromStartGardenPose.ShootPreloads(robot));
        addStep(new FromShootGardenPose.ToIntakeGarden(robot));
        addStep(new FromShootGardenPose.ToIntakeFromWestFlower(robot));
        addStep(new FromShootGardenPose.ToIntakeNorthFlowerToLoading(robot));
        addStep(new FromShootLoadingPose.ToIntakeFromNorthFlower(robot));
        addStep(new FromShootLoadingPose.ToIntakeWestFlowerToGarden(robot));
        addStep(new FromShootGardenPose.ToIntakeGardenToLoading(robot));
        addStep(new FromShootLoadingPose.ToIntakeLoadingToGarden(robot));
        addStep(new FromShootGardenPose.ToIntakeGardenToLoading(robot), " (again)");
        addStep(new FromShootLoadingPose.ToIntakePartnerPreloadsToGarden(robot));
        tourIndex = 0;
        running = false;
        currentPlan = null;
        currentPlanner = null;
        prevA = false;
    }

    @Override
    protected void build(ArchitectureManager manager) {
        // Nothing scheduled up front: each planner is scheduled on an
        // A press in onLoop().
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
        robot.update();

        // Finish detection runs before any new scheduling below, so a plan
        // scheduled this iteration is not checked until the next one.
        if (running && currentPlan != null && !manager.scheduler().isScheduled(currentPlan)) {
            currentPose = currentPlanner.getEndPose();
            running = false;
            currentPlan = null;
            currentPlanner = null;
            tourIndex++;
        }

        // A (rising edge): run the next planner in the tour.
        boolean a = gamepad1.a;
        if (a && !prevA) {
            if (!running && tourIndex < tour.size()) {
                Step step = tour.get(tourIndex);
                currentPlanner = step.planner;
                currentPlanner.setStartPose(currentPose);
                currentPlan = currentPlanner;
                manager.schedule(currentPlan);
                running = true;
            }
        }
        prevA = a;

        telemetry.addData("PlannerTest step", tourIndex + " / " + tour.size());
        if (tourIndex < tour.size()) {
            telemetry.addData("PlannerTest next", tour.get(tourIndex).name);
            telemetry.addData("PlannerTest", "Press A to run it");
        } else {
            telemetry.addData("PlannerTest", "TOUR COMPLETE - stop, re-place the robot at the garden start, and re-init to run again");
        }
        if (currentPlanner != null) {
            telemetry.addData("PlannerTest running", currentPlanner.toString());
        }
        telemetry.addData("PlannerTest plan running", currentPlan == null ? "none" : manager.scheduler().isScheduled(currentPlan));
        telemetry.addData("PlannerTest follower mode", robot.follower.mode());
        telemetry.addData("PlannerTest follower path", robot.follower.pathIndex());
        telemetry.addData("PlannerTest pose", robot.getPose());
    }

    @Override
    public void onStop() {
        if (!isStopped) {
            OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
            cancelPlan();
            robot.stopEverything();
            isStopped = true;
        }
        super.onStop();
    }

    private void cancelPlan() {
        if (currentPlan != null && manager.scheduler().isScheduled(currentPlan)) {
            manager.cancel(currentPlan);
        }
    }
}
