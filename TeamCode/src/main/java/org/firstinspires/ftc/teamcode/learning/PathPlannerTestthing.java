package org.firstinspires.ftc.teamcode.learning;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@Autonomous
public class PathPlannerTestthing extends OpMode {
    Robot robot;
    Command plan;
    @Override
    public void init() {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap, RobotMode.AUTONOMOUS);
        plan = PlanBuilder.startGardenPose(robot)
                .shootPreloads()
                .intakeGarden()
                .schedule();
        robot.build().periodic(new State(RobotMode.AUTONOMOUS)).schedule();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
