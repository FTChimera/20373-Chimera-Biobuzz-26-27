package org.firstinspires.ftc.teamcode.library.planners.paths;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.library.robot.RobotIvyBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;

import static org.firstinspires.ftc.teamcode.library.planners.Common.*;

public class FromShootGardenPose {
    public static class ToIntakeGardenPollen implements PathPlanner {
        volatile Robot robot;
        Pose startPose;
        PathChain to;
        PathChain from;
        public ToIntakeGardenPollen(Robot robot, PathPlanner prevPlanner) {
            this.robot = robot;
            fromPrev(prevPlanner);
        }

        @Override
        public Pose getEndPose() {
            return ShootPoses.garden.pose();
        }

        @Override
        public Command run() {
            RobotIvyBuilder r = robot.build();
            return sequential(
                    r.startIntake(),
                    r.followPath(to),
                    Commands.waitMs(200),
                    r.followPath(from),
                    r.stopIntake()//,
                    // shoot
            );
        }

        @Override
        public void setStartPose(Pose pose) {
            startPose = pose;
        }

        @Override
        public void buildPaths() {
            to = new PathChain(Pedro.linear(ShootPoses.garden.pose(), IntakePoses.gardenControl1.pose(), IntakePoses.gardenControl2.pose(), IntakePoses.gardenEnd.pose())); // todo finish
        }
    }

    public static class ToIntakeGardenPollenToVision extends ToIntakeGardenPollen {
        PathChain toVision;
        public ToIntakeGardenPollenToVision(Robot robot, PathPlanner prevPlanner) {
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
                    robot.build().followPath(toVision)
            );
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            toVision = new PathChain(Pedro.linear(super.getEndPose(), StartPoses.visionAnchorGarden.pose()));
        }
    }
}
