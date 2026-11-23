package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.Plan;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import com.pedropathing.api.Paths;

import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.modules.FromStartLoadingPose;
import org.firstinspires.ftc.teamcode.planners.modules.Utils;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import com.rmsftc.utils.RobotMode;

/**
 * Red loading start: shoot preloads, then follow a body-clear parking route.
 */

@Autonomous(name = "Evergreen Tech A-trons Auto RED", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class EvergreenTechAuto extends ArchitecturalOpMode {
    Robot robot;
    boolean isStopped = false;
    Command plan;
    boolean startParked;
    private boolean started;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    protected void onInit() {
        OpModeStorage.allianceColor.set(AllianceColor.RED);
        OpModeStorage.pose.set(StartPoses.loadingArea);
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setSmartShooterControl(true);
    }

    @Override
    protected void build(ArchitectureManager manager) {
        plan = Plan.sequence(Position.LOADING_START, StartPoses.loadingArea,
                new FromStartLoadingPose.ShootPreloads(robot),
                new Utils.CustomPath(robot,
                        robot.field.generatePathToLoadingZoneShortest(ShootPoses.loading, 3)));
        manager.schedule(plan);
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
        telemetry.addData("Garden12 plan", plan == null ? "not scheduled" : manager.scheduler().isScheduled(plan));
        telemetry.addData("Garden12 follower mode", robot.follower.mode());
        telemetry.addData("Garden12 follower path", robot.follower.pathIndex());

        if ((robot.shouldPark() || (plan != null && !manager.scheduler().isScheduled(plan))) && !startParked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                cancelPlan();
                startParked = robot.moveToParkZone();
            }
        }
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
        cancelPlan();
        robot.stopEverything();
        isStopped = true;
        interrupt();
    }

    private void cancelPlan() {
        if (plan != null && manager.scheduler().isScheduled(plan)) {
            manager.cancel(plan);
        }
    }
}
