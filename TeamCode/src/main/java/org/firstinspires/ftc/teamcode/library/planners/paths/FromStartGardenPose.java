package org.firstinspires.ftc.teamcode.library.planners.paths;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import static org.firstinspires.ftc.teamcode.library.planners.Common.*;

import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;

public class FromStartGardenPose extends Utils.StartPose {
    public FromStartGardenPose(Robot robot) {
        super(robot, StartPoses.garden);
    }

    public static class ShootPreloads implements PathPlanner {
        volatile Robot robot;
        Path[] paths = new Path[2];
        Pose startingPose;
        public ShootPreloads(Robot robot, PathPlanner prevPlanner) {
            this.robot = robot;
            fromPrev(prevPlanner);
        }

        @Override
        public Pose getEndPose() {
            return ShootPoses.garden.pose();
        }

        @Override
        public Command run() {
            return sequential(
                    // start shooter
                    robot.build().followPath(new PathChain(paths[0]))
                    // wait until robot shooter ready
                    // shoot preloads
            );
        }

        @Override
        public void setStartPose(Pose pose) {
            startingPose = pose;
        }

        @Override
        public void buildPaths() {
            paths[0] = Pedro.linear(StartPoses.garden.pose(), ShootPoses.garden.pose());
            paths[1] = null;
        }
    }

    public static class ShootPreloadsToVisionSearch extends ShootPreloads {
        public ShootPreloadsToVisionSearch(Robot robot, PathPlanner prevPlanner) {
            super(robot, prevPlanner);
        }
        @Override
        public Pose getEndPose() {
            return StartPoses.visionAnchorGarden.pose(); // return StartPoses.visionGardenAnchor; or IntakePoses
        }

        @Override
        public Command run() {
            return sequential(
                    super.run(),
                    robot.build().followPath(new PathChain(paths[1]))
            );
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            paths[1] = Pedro.linear(super.getEndPose(), StartPoses.visionAnchorGarden.pose());
        }
    }
}
