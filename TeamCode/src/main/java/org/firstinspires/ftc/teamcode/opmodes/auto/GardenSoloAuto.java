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

/**
 * Garden solo auto: garden start, shoot preloads, garden intake to
 * loading shoot, north flower intake (shoot at loading), loading
 * preloads intake (no-show partner's POLLEN against the loading-zone
 * wall, if present) to garden shoot, west flower intake and garden
 * shoot, then park. The north flower shoot at loading is the second
 * cycle that tips the hive; the garden preload shoot is the first
 * (untipped) cycle.
 */
@Autonomous(name = "Garden Solo Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class GardenSoloAuto extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Command plan;
    boolean startParked;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    public void init() {
        Scheduler.INSTANCE.reset();
        OpModeStorage.pose = StartPoses.gardenArea;
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        robot.setSmartShooterControl(true);
        plan = PlanBuilder.startGarden(robot)
                .shootPreloads()
                .intakeGardenToLoading()
                .intakeFromNorthFlower()
                .intakeLoadingToGarden()
                .intakeFromWestFlower()
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
        telemetry.addData("GardenSolo plan", plan == null ? "not scheduled" : Scheduler.INSTANCE.isRunning(plan));
        telemetry.addData("GardenSolo follower mode", robot.follower.mode());
        telemetry.addData("GardenSolo follower path", robot.follower.pathIndex());
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
