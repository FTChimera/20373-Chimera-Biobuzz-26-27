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
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Garden 8 Ball Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME) //TODO: change to main tele
public class Garden8 extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Timer zeroVelocityTimer = new Timer(TimeUnit.SECONDS);
    double pastTime;
    Command plan;
    boolean parked;

    @Override
    public void init() {
        Scheduler.INSTANCE.reset();
        OpModeStorage.pose = StartPoses.gardenArea;
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
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

        if ((robot.shouldPark() || !Scheduler.INSTANCE.isRunning(plan))&&!parked) {
            robot.moveToParkZone();
            parked = true;
        }

    }

    @Override
    public void stop() {
        if (!isStopped) OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
        Scheduler.INSTANCE.reset();
    }
}
