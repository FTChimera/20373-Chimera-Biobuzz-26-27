package org.firstinspires.ftc.teamcode.tests;


import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.math.MathUtils;
import com.rmsftc.library.planners.PathPlannerList;
import com.rmsftc.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;

public class TestPathPlanner extends LinearOpMode {
    Mechanisms mechanisms;
    PathPlannerList steps;


    @Override
    public void runOpMode() throws InterruptedException {
        mechanisms = Constants.createMechanisms(hardwareMap);
        steps = new PlanBuilder(mechanisms, new Pose(0,0,0))
                .linearPathTo(new Pose(12, 12, MathUtils.DEG_TO_RAD * 90))
                .waitMs(1000)
                .linearPathTo(new Pose(0, 0, MathUtils.DEG_TO_RAD * 0))
                .build();

        steps.schedule();
        telemetry.addData("Scheduled", "Yes");
        telemetry.addLine("---MECHANISMS---");
        mechanisms.all().forEach(mechanism -> telemetry.addLine(mechanism.getClass().getSimpleName()));
        waitForStart();
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
