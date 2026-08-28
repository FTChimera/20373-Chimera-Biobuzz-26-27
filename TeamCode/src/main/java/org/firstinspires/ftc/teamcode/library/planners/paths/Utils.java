package org.firstinspires.ftc.teamcode.library.planners.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.Timer;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Utils {
    public static class StartPose implements PathPlanner {
        Pose pose;
        Robot robot;

        public StartPose(Robot robot, Pose pose) {
            this.robot = robot;
            this.pose = pose;
        }

        @Override
        public Pose getEndPose() {
            return pose;
        }

        @Override
        public Command run() {
            return Commands.instant(() -> robot.require(RobotDrive.class).mecanumDrive.setStartingPose(pose));
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
        Robot robot;
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
        private static final double DETECTOR_TARGET_HEIGHT_INCHES = 1.5;
        private static final double AREA_DISTANCE_SCALE = 18.0;//tune - move to ll
        private static final double MIN_CONFIDENCE = 0.35;
        private static final double MIN_TARGET_AREA = 0.15;
        private static final double MIN_FORWARD_DISTANCE_INCHES = 1.0;
        private static final double MAX_FORWARD_DISTANCE_INCHES = 48.0;
        private static final double MAX_LATERAL_OFFSET_INCHES = 10.0;
        private static final double MERGE_DISTANCE_INCHES = 2.5;
        private static final double PEDRO_CURVE_THROUGH_TENSION = 0.5;
        private static final double ADAPTIVE_HANDLE_FRACTION = 1.0 / 3.0;
        private static final double CURVE_THROUGH_MAX_TURN_RADIANS = Math.toRadians(70);
        private static final double CURVE_THROUGH_MAX_SPACING_RATIO = 1.8;
        private static final double SCORE_CONFIDENCE_WEIGHT = 0.45;
        private static final double SCORE_AREA_WEIGHT = 0.35;
        private static final double SCORE_LATERAL_WEIGHT = 0.20;
        private static final double SCORE_TURN_WEIGHT = 0.90;
        private static final double SCORE_GAP_WEIGHT = 0.04;
        private static final double DETECTOR_RETRY_SECONDS = 0.7;

        Robot robot;
        Vision vision;
        RobotDrive drive;
        Intake intake;
        boolean shouldUseFailSafePath = false;
        PathChain failSafePath;
        int numBalls;
        Pose startPose;
        Pose resolvedEndPose;
        PathChain generatedPath;
        boolean outboundPathStarted, returnPathStarted;
        Timer retryTimer;

        private static class DetectionCandidate {
            final Pose fieldPose;
            final double forwardDistance;
            final double lateralOffset;
            final double qualityScore;

            private DetectionCandidate(Pose fieldPose, double forwardDistance, double lateralOffset, double qualityScore) {
                this.fieldPose = fieldPose;
                this.forwardDistance = forwardDistance;
                this.lateralOffset = lateralOffset;
                this.qualityScore = qualityScore;
            }
        }
//todo add boundaries class and pass it here, add checks
        public IntakeWVision(Robot robot, Vision vision, PathPlanner prevPlanner, int numBalls, PathChain failSafePath) {
            this.robot = robot;
            this.vision = vision;
            this.numBalls = numBalls;
            this.failSafePath = failSafePath;
            setUpMechanisms();
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
                            // First sort - by tA and a little bit of confidence
                            // This means the closer detectors are first.
                            detectors.sort(Comparator.comparingDouble(this::getScorePass1).reversed());
                            PathChain intakePath = buildIntakePath(robotPose, detectors);
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

        private PathChain buildIntakePath(Pose robotPose, List<LLResultTypes.DetectorResult> detectors) {
            if (numBalls <= 0) return null;

            List<DetectionCandidate> candidates = buildCandidates(robotPose, detectors);
            if (candidates.isEmpty()) return null;

            List<DetectionCandidate> selected = selectBestSequence(candidates, Math.min(numBalls, candidates.size()));
            if (selected.isEmpty()) return null;

            return buildSmoothPathChain(robotPose, selected);
        }

        private List<DetectionCandidate> buildCandidates(Pose robotPose, List<LLResultTypes.DetectorResult> detectors) {
            List<DetectionCandidate> candidates = new ArrayList<>();

            for (LLResultTypes.DetectorResult detector : detectors) {
                DetectionCandidate candidate = toCandidate(robotPose, detector);
                if (candidate != null) {
                    candidates.add(candidate);
                }
            }

            candidates.sort(Comparator.comparingDouble((DetectionCandidate candidate) -> candidate.qualityScore).reversed());

            List<DetectionCandidate> deduped = new ArrayList<>();
            for (DetectionCandidate candidate : candidates) {
                boolean duplicate = false;
                for (DetectionCandidate existing : deduped) {
                    if (MathUtils.distance(candidate.forwardDistance, candidate.lateralOffset,
                            existing.forwardDistance, existing.lateralOffset) < MERGE_DISTANCE_INCHES) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    deduped.add(candidate);
                }
            }

            deduped.sort(Comparator.comparingDouble((DetectionCandidate candidate) -> candidate.forwardDistance));
            return deduped;
        }

        private DetectionCandidate toCandidate(Pose robotPose, LLResultTypes.DetectorResult detector) {
            double confidence = detector.getConfidence() / 100.0;
            double area = detector.getTargetArea() / 100.0;
            if (confidence < MIN_CONFIDENCE || area < MIN_TARGET_AREA) return null;

            double forwardDistance = estimateForwardDistanceInches(detector, area);
            if (!Double.isFinite(forwardDistance)
                    || forwardDistance < MIN_FORWARD_DISTANCE_INCHES
                    || forwardDistance > MAX_FORWARD_DISTANCE_INCHES) {
                return null;
            }

            double lateralOffset = forwardDistance * Math.tan(MathUtils.units.toRadians(detector.getTargetXDegrees()));
            if (!Double.isFinite(lateralOffset) || Math.abs(lateralOffset) > MAX_LATERAL_OFFSET_INCHES) return null;

            Pose fieldPose = robotRelativeToField(robotPose, forwardDistance, lateralOffset);
            return new DetectionCandidate(fieldPose, forwardDistance, lateralOffset, scoreCandidate(confidence, area, lateralOffset));
        }

        private double estimateForwardDistanceInches(LLResultTypes.DetectorResult detector, double normalizedArea) {
            double heightDelta = DETECTOR_TARGET_HEIGHT_INCHES - vision.getLensHeightInches();
            double angleRadians = MathUtils.units.toRadians(vision.getMountAngleDegrees() + detector.getTargetYDegrees());
            double tangent = Math.tan(angleRadians);

            if (Math.abs(heightDelta) > MathUtils.EPSILON && Math.abs(tangent) > MathUtils.EPSILON) {
                double tyDistance = heightDelta / tangent;
                if (Double.isFinite(tyDistance) && tyDistance > 0) {
                    return tyDistance;
                }
            }

            return AREA_DISTANCE_SCALE / Math.sqrt(Math.max(normalizedArea, MathUtils.EPSILON));
        }

        private Pose robotRelativeToField(Pose robotPose, double forwardDistance, double lateralOffset) {
            return MathUtils.offsetPoseInRobotFrame(
                    robotPose,
                    forwardDistance,
                    lateralOffset,
                    Constants.FTC_COORDINATE_SYSTEM
            );
        }

        private double scoreCandidate(double confidence, double area, double lateralOffset) {
            double lateralPenalty = Math.min(Math.abs(lateralOffset) / MAX_LATERAL_OFFSET_INCHES, 1.0);
            return SCORE_CONFIDENCE_WEIGHT * confidence
                    + SCORE_AREA_WEIGHT * Math.min(area, 1.0)
                    - SCORE_LATERAL_WEIGHT * lateralPenalty;
        }

        private List<DetectionCandidate> selectBestSequence(List<DetectionCandidate> candidates, int desiredCount) {
            int candidateCount = candidates.size();
            double[][] bestScores = new double[desiredCount + 1][candidateCount];
            double[][] arrivalHeadings = new double[desiredCount + 1][candidateCount];
            int[][] previousIndices = new int[desiredCount + 1][candidateCount];

            for (int sequenceLength = 0; sequenceLength <= desiredCount; sequenceLength++) {
                for (int i = 0; i < candidateCount; i++) {
                    bestScores[sequenceLength][i] = Double.NEGATIVE_INFINITY;
                    previousIndices[sequenceLength][i] = -1;
                }
            }

            for (int i = 0; i < candidateCount; i++) {
                DetectionCandidate candidate = candidates.get(i);
                double headingFromRobot = Math.atan2(candidate.lateralOffset, candidate.forwardDistance);
                bestScores[1][i] = candidate.qualityScore - SCORE_TURN_WEIGHT * Math.abs(headingFromRobot);
                arrivalHeadings[1][i] = headingFromRobot;
            }

            for (int sequenceLength = 2; sequenceLength <= desiredCount; sequenceLength++) {
                for (int current = 0; current < candidateCount; current++) {
                    DetectionCandidate currentCandidate = candidates.get(current);
                    for (int previous = 0; previous < current; previous++) {
                        if (!Double.isFinite(bestScores[sequenceLength - 1][previous])) continue;

                        DetectionCandidate previousCandidate = candidates.get(previous);
                        if (currentCandidate.forwardDistance <= previousCandidate.forwardDistance) continue;

                        double segmentHeading = Math.atan2(
                                currentCandidate.lateralOffset - previousCandidate.lateralOffset,
                                currentCandidate.forwardDistance - previousCandidate.forwardDistance
                        );
                        double turnPenalty = SCORE_TURN_WEIGHT * Math.abs(
                                MathUtils.wrapAngle(segmentHeading - arrivalHeadings[sequenceLength - 1][previous])
                        );
                        double gapPenalty = SCORE_GAP_WEIGHT * MathUtils.distance(
                                currentCandidate.forwardDistance,
                                currentCandidate.lateralOffset,
                                previousCandidate.forwardDistance,
                                previousCandidate.lateralOffset
                        );
                        double score = bestScores[sequenceLength - 1][previous]
                                + currentCandidate.qualityScore
                                - turnPenalty
                                - gapPenalty;

                        if (score > bestScores[sequenceLength][current]) {
                            bestScores[sequenceLength][current] = score;
                            arrivalHeadings[sequenceLength][current] = segmentHeading;
                            previousIndices[sequenceLength][current] = previous;
                        }
                    }
                }
            }

            int bestEndIndex = -1;
            double bestEndScore = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < candidateCount; i++) {
                if (bestScores[desiredCount][i] > bestEndScore) {
                    bestEndScore = bestScores[desiredCount][i];
                    bestEndIndex = i;
                }
            }

            if (bestEndIndex < 0) {
                int bestSingleIndex = -1;
                double bestSingleScore = Double.NEGATIVE_INFINITY;
                for (int i = 0; i < candidateCount; i++) {
                    if (bestScores[1][i] > bestSingleScore) {
                        bestSingleScore = bestScores[1][i];
                        bestSingleIndex = i;
                    }
                }
                if (bestSingleIndex < 0) return new ArrayList<>();

                List<DetectionCandidate> bestSingle = new ArrayList<>();
                bestSingle.add(candidates.get(bestSingleIndex));
                return bestSingle;
            }

            List<DetectionCandidate> bestSequence = new ArrayList<>();
            int index = bestEndIndex;
            for (int sequenceLength = desiredCount; sequenceLength >= 1 && index >= 0; sequenceLength--) {
                bestSequence.add(0, candidates.get(index));
                index = previousIndices[sequenceLength][index];
            }
            return bestSequence;
        }

        private PathChain buildSmoothPathChain(Pose robotPose, List<DetectionCandidate> selected) {
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(new Pose(robotPose.getX(), robotPose.getY()));
            for (DetectionCandidate candidate : selected) {
                waypoints.add(candidate.fieldPose);
            }

            if (waypoints.size() < 2) return null;
            if (waypoints.size() == 2) return buildAdaptiveHandlePathChain(waypoints);

            if (shouldUsePedroCurveThrough(waypoints)) {
                return buildPedroCurveThroughPathChain(waypoints);
            }

            return buildAdaptiveHandlePathChain(waypoints);
        }

        private boolean shouldUsePedroCurveThrough(List<Pose> waypoints) {
            return MathUtils.maxTurnAngle(waypoints) <= CURVE_THROUGH_MAX_TURN_RADIANS
                    && MathUtils.maxSpacingRatio(waypoints) <= CURVE_THROUGH_MAX_SPACING_RATIO;
        }

        private PathChain buildPedroCurveThroughPathChain(List<Pose> waypoints) {
            Pose start = waypoints.get(0);
            Pose mirroredPreviousPoint = start.minus(waypoints.get(1).minus(start));
            Pose[] followPoints = waypoints.subList(1, waypoints.size()).toArray(new Pose[0]);

            PathChain pathChain = drive.mecanumDrive.follower.pathBuilder()
                    .curveThrough(mirroredPreviousPoint, start, PEDRO_CURVE_THROUGH_TENSION, followPoints)
                    .build();

            for (int i = 0; i < pathChain.size(); i++) {
                pathChain.getPath(i).setTangentHeadingInterpolation();
            }
            pathChain.setDecelerationType(PathChain.DecelerationType.LAST_PATH);
            resolvedEndPose = pathChain.endPose();
            return pathChain;
        }

        private PathChain buildAdaptiveHandlePathChain(List<Pose> waypoints) {
            Pose[] tangents = MathUtils.smoothTangents(waypoints);
            double[] handleLengths = MathUtils.adaptiveHandleLengths(waypoints, ADAPTIVE_HANDLE_FRACTION);
            List<Path> segments = new ArrayList<>();
            for (int i = 0; i < waypoints.size() - 1; i++) {
                Pose start = waypoints.get(i);
                Pose end = waypoints.get(i + 1);
                double segmentDistance = start.distanceFrom(end);
                if (segmentDistance < MathUtils.EPSILON) continue;

                Pose control1 = MathUtils.addScaled(start, tangents[i], handleLengths[i]);
                Pose control2 = MathUtils.addScaled(end, tangents[i + 1], -handleLengths[i + 1]);

                Path segment = new Path(new BezierCurve(start, control1, control2, end));
                segment.setTangentHeadingInterpolation();
                segments.add(segment);
            }

            if (segments.isEmpty()) return null;
            PathChain pathChain = new PathChain(segments.toArray(new Path[0]));
            pathChain.setDecelerationType(PathChain.DecelerationType.LAST_PATH);
            resolvedEndPose = pathChain.endPose();
            return pathChain;
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
