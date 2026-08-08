package org.firstinspires.ftc.teamcode.library.planners.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

public class Utils {
    public static class StartPose implements PathPlanner {
        Pose pose;
        Mechanisms mechanisms;
        public StartPose(Mechanisms mechanisms, Pose pose) {
            this.mechanisms = mechanisms;
            this.pose = pose;
        }
        @Override
        public Pose getEndPose() {
            return pose;
        }

        @Override
        public Command run() {
            return Commands.instant(() -> mechanisms.require(RobotDrive.class).mecanumDrive.setStartingPose(pose));
        }

        @Override
        public void setStartPose(Pose pose) {
            this.pose = pose;
        }

        @Override
        public void buildPaths() {
            // no need to do anything
        }

        @Override
        public void fromPrev(PathPlanner prev) {
            setStartPose(prev.getEndPose());
        }
    }
    public static class CustomPath implements PathPlanner {
        Mechanisms mechanisms;
        PathChain pathChain;
        public CustomPath(Mechanisms mechanisms, PathPlanner prevPlanner, PathChain pathChain) {
            this.mechanisms = mechanisms;
            this.pathChain = pathChain;
            setStartPose(prevPlanner.getEndPose());
        }
        @Override
        public Pose getEndPose() {
            return pathChain.endPose();
        }

        @Override
        public Command run() {
            return PedroCommands.follow(mechanisms.require(RobotDrive.class).mecanumDrive.follower, pathChain);
        }

        @Override
        public void setStartPose(Pose pose) {
        }

        @Override
        public void buildPaths() {
        }

        @Override
        public void fromPrev(PathPlanner prev) {
            setStartPose(prev.getEndPose());
        }

        public static class IntakeWVision implements PathPlanner {

            @Override
            public Pose getEndPose() {
                return null;
            }

            @Override
            public Command run() {
                return null;
            }

            @Override
            public void setStartPose(Pose pose) {

            }

            @Override
            public void buildPaths() {

            }

            @Override
            public void fromPrev(PathPlanner prev) {
                setStartPose(prev.getEndPose());
            }
        }
    }
}
