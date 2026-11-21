package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.commands.Scheduler;
import org.firstinspires.ftc.teamcode.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

/**
 * Garden start, shoot preloads (4), garden intake and shoot (4), west
 * flower intake and shoot (4), then park. All three shoots are at the
 * garden shoot pose with the hive UNTIPPED, so no cross-field legs.
 */
@Autonomous(name = "Evergreen Tech A-trons Auto RED", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class EvergreenTechAuto extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Command plan;
    boolean startParked;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    public void init() {
        Scheduler.INSTANCE.reset();
        OpModeStorage.allianceColor = AllianceColor.RED;
        OpModeStorage.pose = StartPoses.loadingArea;
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        robot.setSmartShooterControl(true);
        plan = PlanBuilder.startLoading(robot)
                .shootPreloads()
                .customPath(Paths.line(ShootPoses.loading, PoseFactory.deg(14.26, 96, 90)).constant(90))
                .schedule();
    }

    @Override public void start() {robot.start();}


    @Override
    public void loop() {
        if (robot.opModeTimeRemaining() <= 0 || robot.inParkZone()) {
            stopAutonomous();
            return;
        }

        Scheduler.INSTANCE.execute();
        robot.update();
        telemetry.addData("Garden12 plan", plan == null ? "not scheduled" : Scheduler.INSTANCE.isRunning(plan));
        telemetry.addData("Garden12 follower mode", robot.follower.mode());
        telemetry.addData("Garden12 follower path", robot.follower.pathIndex());
        telemetry.update();

        if ((robot.shouldPark() || (plan != null && !Scheduler.INSTANCE.isRunning(plan)))&&!startParked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                cancelPlan();
//                startParked = robot.moveToParkZone();
            }
        }

    }

    @Override
    public void stop() {
        if (!isStopped) {
            OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
            cancelPlan();
            robot.stopEverything();
        }
        Scheduler.INSTANCE.reset();
    }

    private void stopAutonomous() {
        if (isStopped) {
            return;
        }
        OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
        cancelPlan();
        robot.stopEverything();
        isStopped = true;
        requestOpModeStop();
    }

    private void cancelPlan() {
        if (plan != null && Scheduler.INSTANCE.isScheduled(plan)) {
            Scheduler.INSTANCE.cancel(plan);
        }
    }
}
