package org.firstinspires.ftc.teamcode.learning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

public class ExampleIvyAuto extends OpMode {
    private Follower follower;

    public static final Pose startPose = new Pose(
            123.5,
            122.8,
            Math.toRadians(37.5)
    );

    public static final Pose launchPose = new Pose(
            108,
            108,
            Math.toRadians(45)
    );

    public static final Pose set_1Pose = new Pose(
            96,
            84,
            Math.toRadians(0)
    );

    public PathChain launchPath;
    public PathChain set_1Path;

    public void buildPaths() {
        launchPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose, launchPose
                ))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(), launchPose.getHeading()
                )
                .build();

        set_1Path = follower.pathBuilder()
                .addPath(new BezierLine(
                        launchPose, set_1Pose
                ))
                .setLinearHeadingInterpolation(
                        launchPose.getHeading(), set_1Pose.getHeading()
                )
                .build();
    }

    @Override
    public void init() {
        reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        schedule(
                infinite(follower::update),
                sequential(
                        follow(launchPath),
                        follow(set_1Path)
                )
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    @Override
    public void stop() {
        reset();
    }

    // Helper methods
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }
    public Command follow(PathChain pathChain) {
        return PedroCommands.follow(follower, pathChain);
    }
    public void reset() {
        Scheduler.reset();
    }
    public Command infinite(Runnable run) {
        return Commands.infinite(run);
    }
    public Command sequential(Command... commands) {
        return Groups.sequential(commands);
    }
}
