package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Garden 8 Ball Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
@Config
public class Garden8 extends OpMode {
    public static int PRELOAD_SHOOTER_VELOCITY_TICKS_PER_SECOND = 0;
    Robot robot;
    boolean isStopped=false;
    Timer zeroVelocityTimer = new Timer(TimeUnit.SECONDS);
    double pastTime;
    Command plan;
    boolean parked;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    public void init() {
        Scheduler.INSTANCE.reset();
        OpModeStorage.pose = StartPoses.gardenArea;
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        if (PRELOAD_SHOOTER_VELOCITY_TICKS_PER_SECOND <= 0) {
            telemetry.addLine("Set PRELOAD_SHOOTER_VELOCITY_TICKS_PER_SECOND before starting autonomous.");
            telemetry.update();
            return;
        }
        robot.shooter.setVelocity(PRELOAD_SHOOTER_VELOCITY_TICKS_PER_SECOND);
        plan = PlanBuilder.startGarden(robot)
                .shootPreloads()
                .intakeFromGarden()
                .schedule();
    }

    @Override public void start() {robot.start();}


    @Override
    public void loop() {
        Scheduler.INSTANCE.execute();
        robot.update();
        // todo: implement logging for plan
        telemetry.update();
        if (robot.opModeTimeRemaining() <= 0) {
            if (!isStopped) {
                zeroVelocityTimer.reset();
                pastTime = 2;
                robot.stopAllSubsystems();
                isStopped = true;
            }
            if (robot.getVelocity().toVector().magnitude() < 0.2 || zeroVelocityTimer.past(pastTime)) {
                // save end vals
                OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
                requestOpModeStop();
            }
        }

        if ((robot.shouldPark() || (plan != null && !Scheduler.INSTANCE.isRunning(plan)))&&!parked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                parked = robot.moveToParkZone();
            }
        }

    }

    @Override
    public void stop() {
        if (!isStopped) OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
        Scheduler.INSTANCE.reset();
    }
}
