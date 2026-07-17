package org.firstinspires.ftc.teamcode.learning.teachEason;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

public class SkillBuilder7 extends LinearOpMode {
    public Pose startPose = new Pose(0, 72, 0);
    public Pose zone1Pose = new Pose(120, 120, 0);
    public Pose zone2Pose = new Pose(120, 72, 0);
    public Pose zone3Pose = new Pose(120, 5, 0);

    public PathChain zone1, zone1rev, zone2, zone2rev, zone3, zone3rev;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        zone1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, zone1Pose))
                .setConstantHeadingInterpolation(0)
                .build();

        zone1rev = new PathChain(zone1.firstPath().getReversed());

        zone2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, zone2Pose))
                .setConstantHeadingInterpolation(0)
                .build();

        zone2rev = new PathChain(zone2.firstPath().getReversed());
    }
}
