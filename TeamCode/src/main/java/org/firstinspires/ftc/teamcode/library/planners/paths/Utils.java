package org.firstinspires.ftc.teamcode.library.planners.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

import java.util.Comparator;
import java.util.List;

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
        int numBalls;
        Pose startPose;

        public IntakeWVision(Mechanisms mechanisms, PathPlanner prevPlanner, int numBalls, PathChain failSafePath) {
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
        public void buildPaths() {
            // we build paths on fly
            // set pipeline to make sure it is not in april tag mode.
            limelight.setPipeline(Limelight.Pipeline.NEURAL_DETECTOR);
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
                                    // Switch pipeline to neural detector again
                                    // because pipeline might have been switched back to april tag localization.
                                    limelight.setPipeline(Limelight.Pipeline.NEURAL_DETECTOR);
                                    limelight.LLUpdate();

                                    List<LLResultTypes.DetectorResult> detectors = limelight.getDetectors();
                                    if (detectors == null || detectors.isEmpty()) {
                                        // No balls detected, use fail safe path
                                        shouldUseFailSafePath = true;
                                        drive.mecanumDrive.follower.followPath(failSafePath);
                                    } else {
                                        // First sort - by tA and a little bit of confidence
                                        // This means the closer detectors are first.
                                        detectors.sort(
                                                (o1, o2) -> getScorePass1(o1) - getScorePass1(o2) > 0 ? 1 : getScorePass1(o1) == getScorePass1(o2) ? 0 : -1
                                        );

                                    }
                                }
                            }
                    )
                    .setDone(() -> !drive.mecanumDrive.follower.isBusy());
        }

        /** Returns score from 0 to 1 of the result
         * Calculated as a function of confidence, distance (area)
         * @param result
         * @return score, from 0-1
         */
        public double getScorePass1(LLResultTypes.DetectorResult result) {
            double confidence = result.getConfidence()/100; // From 0 to 1
            double tA = result.getTargetArea()/100;
            double confidenceScalar = 1; // Tune this value
            double tAScalar = 1.2; // Tune this value
            return 0.8*tA*tAScalar + 0.2*confidence*confidenceScalar;
        }

        @Override
        public void setStartPose(Pose pose) {
            this.startPose = pose;
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
