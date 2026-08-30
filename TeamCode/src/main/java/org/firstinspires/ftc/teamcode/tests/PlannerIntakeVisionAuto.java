package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.planners.PathPlannerList;
import org.firstinspires.ftc.teamcode.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@Autonomous(name = "Planner Intake Vision Auto", group = "1: Tests")
public class PlannerIntakeVisionAuto extends LinearOpMode {
    private static final Pose START_POSE = new Pose(36, 12, Math.toRadians(180));
    private static final Pose STAGING_POSE = new Pose(28, 12, Math.toRadians(180));
    private static final Pose VISION_ANCHOR_POSE = new Pose(18, 24, Math.toRadians(180));
    private static final Pose VISION_SEARCH_POSE = new Pose(10, 33, Math.toRadians(180));
    private static final Pose PARK_POSE = new Pose(36, 36, Math.toRadians(90));

    private Robot robot;
    private RobotDrive drive;
    private Intake intake;
    private PathPlannerList plan;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap, RobotMode.AUTONOMOUS);
        drive = robot.require(RobotDrive.class);
        intake = robot.get(Intake.class);
        plan = new PlanBuilder(robot, START_POSE)
                .linearPathTo(STAGING_POSE)
                .followPath(createApproachPath())
                .waitMs(250)
                .intakeWVision(2, createVisionFailSafePath())
                .linearPathTo(PARK_POSE)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mechanisms", robot.getConfiguredMechanismClasses());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            Scheduler.reset();
            return;
        }

        robot.start();
        robot.startVision(Vision.Pipeline.NEURAL_DETECTOR);

        plan.schedule();

        while (opModeIsActive()) {
            Scheduler.execute();
            drive.periodic(null, RobotMode.AUTONOMOUS);

            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Plan Steps", plan.size());
            telemetry.addData("Intake Power", intake == null ? 0 : intake.getTargetPower());
            telemetry.update();
        }

        Scheduler.reset();
    }

    private PathChain createApproachPath() {
        Path approach = new Path(new BezierCurve(
                STAGING_POSE,
                new Pose(24, 12),
                new Pose(21, 18),
                VISION_ANCHOR_POSE
        ));
        approach.setHeadingInterpolation(HeadingInterpolator.linear(
                STAGING_POSE.getHeading(),
                VISION_ANCHOR_POSE.getHeading()
        ));
        return new PathChain(approach);
    }

    private PathChain createVisionFailSafePath() {
        Path search = new Path(new BezierLine(VISION_ANCHOR_POSE, VISION_SEARCH_POSE));
        search.setHeadingInterpolation(HeadingInterpolator.linear(
                VISION_ANCHOR_POSE.getHeading(),
                VISION_SEARCH_POSE.getHeading()
        ));
        return new PathChain(search);
    }
}
