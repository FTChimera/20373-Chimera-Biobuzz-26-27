package org.firstinspires.ftc.teamcode.library.planners.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
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
    }

    public static class CustomPath implements PathPlanner {
        Mechanisms mechanisms;
        PathChain pathChain;

        public CustomPath(Mechanisms mechanisms, PathPlanner prevPlanner, PathChain pathChain) {
            this.mechanisms = mechanisms;
            this.pathChain = pathChain;
            fromPrev(prevPlanner);
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
    }
    public static class IntakeWVision implements PathPlanner {
        Mechanisms mechanisms;
        Limelight limelight;
        RobotDrive drive;
        boolean shouldUseFailSafePath = false;
        PathChain failSafePath;
        double numBalls;
        Pose startPose;

        public IntakeWVision(Mechanisms mechanisms, PathPlanner prevPlanner, double numBalls, PathChain failSafePath) {
            this.mechanisms = mechanisms;
            this.numBalls = numBalls;
            this.failSafePath = failSafePath;
            setUpMechanisms();
            fromPrev(prevPlanner);
        }

        public void setUpMechanisms() {
            limelight = mechanisms.get(Limelight.class);
            drive = mechanisms.get(RobotDrive.class);
            shouldUseFailSafePath = limelight == null || drive == null;
        }

        @Override
        public Pose getEndPose() {
            return startPose; // Come back to start pose.
        }

                                        
        @Override
        public Command run() {
            return Command.build()
                    .setStart(() ->
                            {
                                if (shouldUseFailSafePath) {
                                    drive.mecanumDrive.follower.followPath(failSafePath);
                                } else {
                                    // Implement vision-based intake logic here
                                    // Both limelight and drive are not null
                                }
                            }
                    )
                    .setDone(() -> !drive.mecanumDrive.follower.isBusy());
        }

        @Override
        public void setStartPose(Pose pose) {
            this.startPose = pose;
        }

        @Override
        public void buildPaths() {
            // we build paths on fly
        }

    }

    public static class LinearPath extends CustomPath {
        public LinearPath(Mechanisms mechanisms, PathPlanner prevPlanner, Pose endPose) {
            super(mechanisms, prevPlanner,
                    new PathChain(new Path(new BezierLine(
                            prevPlanner.getEndPose(), endPose
                    )
                    )));
        }
    }

    public static class Wait implements PathPlanner {
        Pose startPose;
        long timeMs;

        public Wait(PathPlanner prevPlanner, long timeMs) {
            fromPrev(prevPlanner);
            this.timeMs = timeMs;
        }

        @Override
        public Pose getEndPose() {
            return startPose;
        }

        @Override
        public Command run() {
            return Commands.waitMs(timeMs);
        }

        @Override
        public void setStartPose(Pose pose) {
            this.startPose = pose;
        }

        @Override
        public void buildPaths() {
            // no need to do anything
        }
    }
}
