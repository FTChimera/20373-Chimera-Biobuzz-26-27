package org.firstinspires.ftc.teamcode.library.planners.paths;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.library.IntakeBallPath;
import org.firstinspires.ftc.teamcode.library.Timer;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Utils {
    public static class StartPose extends NullPlanner {
        public StartPose(Robot robot, Pose pose) {
            super(robot, pose);
        }
        @Override public Command run() {
            return Commands.instant(() -> robot.require(RobotDrive.class).mecanumDrive.setStartingPose(pose));
        }
    }
    public static class NullPlanner implements PathPlanner {
        Pose pose;
        volatile Robot robot;

        public NullPlanner(Robot robot, Pose pose) {
            this.robot = robot;
            this.pose = pose;
        }

        @Override
        public Pose getEndPose() {
            return pose;
        }

        @Override
        public Command run() {
            return Command.NOOP;
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
        volatile Robot robot;
        PathChain pathChain;

        public CustomPath(Robot robot, PathPlanner prevPlanner, PathChain pathChain) {
            this.robot = robot;
            this.pathChain = pathChain;
            fromPrev(prevPlanner);
        }

        @Override
        public Pose getEndPose() {
            return pathChain.endPose();
        }

        @Override
        public Command run() {
            return PedroCommands.follow(robot.require(RobotDrive.class).mecanumDrive.follower, pathChain);
        }

        @Override
        public void setStartPose(Pose pose) {
        }

        @Override
        public void buildPaths() {
        }
    }
    public static class IntakeWVision implements PathPlanner {
        private static final double DETECTOR_RETRY_SECONDS = 0.7;

        volatile Robot robot;
        volatile Vision vision;
        volatile RobotDrive drive;
        volatile Intake intake;
        boolean shouldUseFailSafePath = false;
        PathChain failSafePath;
        int numBalls;
        Pose startPose;
        Pose resolvedEndPose;
        PathChain generatedPath;
        boolean outboundPathStarted, returnPathStarted;
        Timer retryTimer;

//todo add boundaries class and pass it here, add checks
        public IntakeWVision(Robot robot, PathPlanner prevPlanner, int numBalls, PathChain failSafePath) {
            this.robot = robot;
            this.vision = robot.requireVision();
            this.numBalls = numBalls;
            this.failSafePath = failSafePath;
            fromPrev(prevPlanner);
        }

        public void setUpMechanisms() {
            drive = robot.get(RobotDrive.class);
            intake = robot.get(Intake.class);
            shouldUseFailSafePath = vision == null || drive == null || intake == null;
        }

        @Override
        public Pose getEndPose() {
            return startPose;
        }
        @Override
        public void buildPaths() {
            // we build paths on fly
            // set pipeline to make sure it is not in april tag mode.
            if (vision != null) {
                vision.setPipeline(Vision.Pipeline.NEURAL_DETECTOR);
            }
        }
        @Override
        public Command run() {
            if (drive == null) return Command.NOOP;

            return Command.build()
                    .setStart(() ->
                            {
                                generatedPath = null;
                                retryTimer = new Timer(TimeUnit.SECONDS);
                                retryTimer.reset();
                                outboundPathStarted = false;
                                returnPathStarted = false;

                                if (shouldUseFailSafePath) {
                                    startFailSafePath();
                                } else if (vision != null) {
                                    // Switch back because the pipeline may have been changed for localization.
                                    vision.setPipeline(Vision.Pipeline.NEURAL_DETECTOR);
                                } else {
                                    startFailSafePath();
                                }
                            }
                    )
                    .setExecute(() -> {
                        if (outboundPathStarted) {
                            if (!returnPathStarted && !drive.mecanumDrive.follower.isBusy()) {
                                startReturnPath(drive.mecanumDrive.follower.getPose());
                            }
                            return;
                        }
                        if (returnPathStarted || shouldUseFailSafePath || vision == null) return;

                        Pose robotPose = drive.mecanumDrive.follower.getPose();
                        vision.updateRobotOrientation(robotPose.getHeading());
                        vision.update();

                        List<LLResultTypes.DetectorResult> detectors = vision.getDetectors();
                        if (detectors != null && !detectors.isEmpty()) {
                            PathChain intakePath = IntakeBallPath.generateIntakePathFromCam(
                                    drive.mecanumDrive.follower,
                                    robotPose,
                                    vision,
                                    numBalls,
                                    detectors
                            );
                            if (intakePath != null) {
                                startOutboundPath(intakePath);
                                return;
                            }
                        }

                        if (retryTimer.get() >= DETECTOR_RETRY_SECONDS) {
                            startFailSafePath();
                        }
                    })
                    .setDone(() -> returnPathStarted && !drive.mecanumDrive.follower.isBusy());
        }

        @Override
        public void setStartPose(Pose pose) {
            this.startPose = pose;
            if (failSafePath != null) {
                resolvedEndPose = failSafePath.endPose();
            } else if (resolvedEndPose == null) {
                resolvedEndPose = pose;
            }
        }

        private void startFailSafePath() {
            if (failSafePath == null) {
                startReturnPath(drive.mecanumDrive.follower.getPose());
                return;
            }
            startOutboundPath(failSafePath);
        }

        private void startOutboundPath(PathChain pathChain) {
            generatedPath = pathChain;
            resolvedEndPose = pathChain.endPose();
            outboundPathStarted = true;
            if (intake != null) intake.FullPower();
            drive.mecanumDrive.follower.followPath(pathChain);
        }

        private void startReturnPath(Pose currentPose) {
            returnPathStarted = true;
            PathChain returnPath = buildReturnPath(currentPose);
            if (intake != null) intake.Stop();
            if (returnPath != null) {
                drive.mecanumDrive.follower.followPath(returnPath);
            }
        }

        private PathChain buildReturnPath(Pose currentPose) {
            if (currentPose.distanceFrom(startPose) < MathUtils.EPSILON) return null;

            double headingToStart = Math.atan2(startPose.getY() - currentPose.getY(), startPose.getX() - currentPose.getX());
            double reverseTangentHeading = MathUtils.wrapAngle(headingToStart + Math.PI);

            return drive.mecanumDrive.follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, startPose))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, 0.5, HeadingInterpolator.tangent.reverse()),
                                    new HeadingInterpolator.PiecewiseNode(0.5, 1, HeadingInterpolator.linear(
                                            reverseTangentHeading,
                                            startPose.getHeading()
                                    ))
                            )
                    )
                    .build();
        }

    }

    public static class LinearPath extends CustomPath {
        public LinearPath(Robot robot, PathPlanner prevPlanner, Pose endPose) {
            super(robot, prevPlanner,
                    new PathChain(new Path(new BezierLine(
                            prevPlanner.getEndPose(), endPose
                    )
                    )));
        }
    }

    public static class Wait implements PathPlanner {
        Pose startPose;
        double timeMs;

        public Wait(PathPlanner prevPlanner, double timeMs) {
            fromPrev(prevPlanner);
            this.timeMs = timeMs;
        }

        @Override
        public Pose getEndPose() {
            return startPose;
        }

        @Override
        public Command run() {
            return Commands.waitMs(Math.round(timeMs));
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
