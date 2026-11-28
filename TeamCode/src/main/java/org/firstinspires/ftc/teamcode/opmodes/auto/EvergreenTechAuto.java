package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.Plan;
import org.firstinspires.ftc.teamcode.planners.Position;
import com.pedropathing.api.Paths;

import org.firstinspires.ftc.teamcode.planners.modules.FromStartLoadingPose;
import org.firstinspires.ftc.teamcode.planners.modules.Utils;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import com.rmsftc.utils.RobotMode;

/**
 * Red loading start: shoot preloads, then follow a body-clear parking route.
 * Assumes the alliance partner tips the hive once at the beginning of auto
 * at garden, so the loading shoot (TIPPED) works from the start.
 */

@Disabled
@Autonomous(name = "Evergreen Tech A-trons Auto RED", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class EvergreenTechAuto extends ArchitecturalOpMode {
    Robot robot;
    boolean isStopped = false;
    Command plan;
    boolean startParked;
    private boolean started;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;

    @Override
    protected void onInit() {
        OpModeStorage.allianceColor.set(AllianceColor.RED);
        OpModeStorage.pose.set(org.firstinspires.ftc.teamcode.planners.Constants.StartPoses.loadingArea);
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setSmartShooterControl(true);
    }

    @Override
    protected void build(ArchitectureManager manager) {
        plan = Plan.sequence(Position.LOADING_START, org.firstinspires.ftc.teamcode.planners.Constants.StartPoses.loadingArea,
                new FromStartLoadingPose.ShootPreloads(robot),
                new Utils.CustomPath(robot,
                        robot.field.generatePathToLoadingZoneShortest(org.firstinspires.ftc.teamcode.planners.Constants.ShootPoses.loading, 3)));
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
        if (robot.opModeTimeRemaining() <= 0 || (startParked&&robot.inParkZone()&&robot.getVelocity().toVector2D().magnitude()<=0.2)) {
            stopAutonomous();
            return;
        }

        robot.update();
        telemetry.addData("Garden12 plan", plan == null ? "not scheduled" : manager.scheduler().isScheduled(plan));
        telemetry.addData("Garden12 follower mode", robot.follower.mode());
        telemetry.addData("Garden12 follower path", robot.follower.pathIndex());

        if ((robot.shouldPark() || (plan != null && !manager.scheduler().isScheduled(plan))) && !startParked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= org.firstinspires.ftc.teamcode.planners.Constants.Robot.Park.RETRY_INTERVAL_SEC) {
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
