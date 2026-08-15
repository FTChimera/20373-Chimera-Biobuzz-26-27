package org.firstinspires.ftc.teamcode.opModes.variants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.rmsftc.library.AllianceColor;
import org.firstinspires.ftc.teamcode.opModes.IvyAuton;

@Autonomous
public class Test_IvyAuton extends IvyAuton {
    // THIS IS A TESTER AUTON FILE
    // DUPLICATE THIS FOR RED AND BLUE ALLIANCE, JUST REMEMBER TO CHANGE THE ALLIANCE COLOR AND PATHS
    static class Paths extends IvyAuton_Paths {
        public static Paths INSTANCE = new Paths();
        Pose STARTING_POSE = new Pose(36, 12, Math.toRadians(180));
        Pose endPose = new Pose(36, 36, Math.toRadians(180));

        @Override
        public PathChain getPath1() {
            Path path1 = new Path(new BezierLine(STARTING_POSE, endPose));
            path1.setHeadingInterpolation(HeadingInterpolator.linear(STARTING_POSE.getHeading(), endPose.getHeading()));
            return new PathChain(path1);
        }

        @Override
        public Pose getStartingPose() {
            return STARTING_POSE.copy();
        }
    }
    public Test_IvyAuton() {
        super(AllianceColor.RED, Paths.INSTANCE);
    }

}
