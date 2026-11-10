package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.commands.Scheduler;
import org.firstinspires.ftc.teamcode.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.RobotMode;

@Autonomous(name = "Loading 8 Ball Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class Loading8 extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Command plan;
    boolean startParked;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    public void init() {
        Scheduler.INSTANCE.reset();
        OpModeStorage.pose = StartPoses.loadingArea;
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        robot.setSmartShooterControl(true);
        plan = PlanBuilder.startLoading(robot)
                .shootPreloads()
                .intakeFromNorthFlower()
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
        telemetry.addData("Loading plan", plan == null ? "not scheduled" : Scheduler.INSTANCE.isRunning(plan));
        telemetry.addData("Loading follower mode", robot.follower.mode());
        telemetry.addData("Loading follower path", robot.follower.pathIndex());
        telemetry.update();

        if ((robot.shouldPark() || (plan != null && !Scheduler.INSTANCE.isRunning(plan)))&&!startParked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                cancelPlan();
                startParked = robot.moveToParkZone();
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
