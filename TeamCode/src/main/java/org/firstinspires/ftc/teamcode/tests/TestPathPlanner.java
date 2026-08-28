package org.firstinspires.ftc.teamcode.tests;


import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.planners.PathPlannerList;
import org.firstinspires.ftc.teamcode.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@Utility
public class TestPathPlanner extends LinearOpMode {
    Robot robot;
    PathPlannerList steps;
    State autonomousState;


    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap, RobotMode.AUTONOMOUS);
        autonomousState = new State(RobotMode.AUTONOMOUS);
        steps = new PlanBuilder(robot, new Pose(0,0,0))
                .linearPathTo(new Pose(12, 12, MathUtils.DEG_TO_RAD * 90))
                .waitMs(1000)
                .linearPathTo(new Pose(0, 0, MathUtils.DEG_TO_RAD * 0))
                .build();

        steps.schedule();
        telemetry.addData("Scheduled", "Yes");
        telemetry.addLine("---MECHANISMS---");
        robot.all().forEach(mechanism -> telemetry.addLine(mechanism.getClass().getSimpleName()));
        waitForStart();
        robot.start();
        while (opModeIsActive()) {
            Scheduler.execute();
            robot.periodic(autonomousState);
        }
        Scheduler.reset();
    }
}
