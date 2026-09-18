package org.firstinspires.ftc.teamcode.library;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.Pollen;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

public final class IntakeBallPath {
    public static final double AREA_DISTANCE_SCALE = 18.0;//tune - move to ll
    public static final double MIN_CONFIDENCE = 0.35;
    public static final double MIN_TARGET_AREA = 0.15;
    public static final double MIN_FORWARD_DISTANCE_INCHES = 1.0;
    public static final double MAX_FORWARD_DISTANCE_INCHES = 48.0;
    public static final double MAX_LATERAL_OFFSET_INCHES = 10.0;
    public static final double MERGE_DISTANCE_INCHES = 2.5;
    public static final double PEDRO_CURVE_THROUGH_TENSION = 0.5;
    public static final double ADAPTIVE_HANDLE_FRACTION = 1.0 / 3.0;
    public static final double CURVE_THROUGH_MAX_TURN_RADIANS = Math.toRadians(70);
    public static final double CURVE_THROUGH_MAX_SPACING_RATIO = 1.8;
    public static final double SCORE_CONFIDENCE_WEIGHT = 0.45;
    public static final double SCORE_AREA_WEIGHT = 0.35;
    public static final double SCORE_LATERAL_WEIGHT = 0.20;
    public static final double SCORE_TURN_WEIGHT = 0.90;
    public static final double SCORE_GAP_WEIGHT = 0.04;

    private IntakeBallPath() {
    }

    public static PathChain generateIntakePathFromCam(
            Follower follower,
            Pose robotPose,
            Vision vision,
            int numBalls,
            List<LLResultTypes.DetectorResult> detectors
    ) {
        if (numBalls <= 0) return null;
        if (detectors == null || detectors.isEmpty()) return null;

        detectors.sort(Comparator.comparingDouble(IntakeBallPath::getScorePass1).reversed());
        List<DetectionCandidate> candidates = buildCandidates(
                robotPose,
                buildReadings(detectors),
                vision.getLensHeightInches(),
                vision.getMountAngleDegrees()
        );
        if (candidates.isEmpty()) return null;

        List<DetectionCandidate> selected = selectBestSequence(candidates, Math.min(numBalls, candidates.size()));
        if (selected.isEmpty()) return null;

        return buildSmoothPathChain(follower, robotPose, selected);
    }

    public static PathChain generateIntakePathFromCam() {
        return new PathChain();
    }

    private static double getScorePass1(LLResultTypes.DetectorResult result) {
        double confidence = result.getConfidence() / 100;
        double tA = result.getTargetArea() / 100;
        double confidenceScalar = 1;
        double tAScalar = 1.2;
        return 0.8 * tA * tAScalar + 0.2 * confidence * confidenceScalar;
    }

    private static List<DetectionReading> buildReadings(List<LLResultTypes.DetectorResult> detectors) {
        List<DetectionReading> readings = new ArrayList<>(detectors.size());
        for (LLResultTypes.DetectorResult detector : detectors) {
            readings.add(new DetectionReading(
                    detector.getConfidence(),
                    detector.getTargetArea(),
                    detector.getTargetXDegrees(),
                    detector.getTargetYDegrees()
            ));
        }
        return readings;
    }

    static List<DetectionCandidate> buildCandidates(
            Pose robotPose,
            List<DetectionReading> detections,
            double lensHeightInches,
            double mountAngleDegrees
    ) {
        List<DetectionCandidate> candidates = new ArrayList<>();

        for (DetectionReading detection : detections) {
            DetectionCandidate candidate = toCandidate(robotPose, detection, lensHeightInches, mountAngleDegrees);
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

    private static DetectionCandidate toCandidate(
            Pose robotPose,
            DetectionReading detection,
            double lensHeightInches,
            double mountAngleDegrees
    ) {
        double confidence = detection.confidencePercent / 100.0;
        double area = detection.targetAreaPercent / 100.0;
        if (confidence < MIN_CONFIDENCE || area < MIN_TARGET_AREA) return null;

        double forwardDistance = estimateForwardDistanceInches(detection.targetYDegrees, area, lensHeightInches, mountAngleDegrees);
        if (!Double.isFinite(forwardDistance)
                || forwardDistance < MIN_FORWARD_DISTANCE_INCHES
                || forwardDistance > MAX_FORWARD_DISTANCE_INCHES) {
            return null;
        }

        double lateralOffset = forwardDistance * Math.tan(MathUtils.units.toRadians(detection.targetXDegrees));
        if (!Double.isFinite(lateralOffset) || Math.abs(lateralOffset) > MAX_LATERAL_OFFSET_INCHES) return null;

        Pose fieldPose = robotRelativeToField(robotPose, forwardDistance, lateralOffset);
        return new DetectionCandidate(fieldPose, forwardDistance, lateralOffset,
                scoreCandidate(confidence, area, lateralOffset));
    }

    static double estimateForwardDistanceInches(
            double targetYDegrees,
            double normalizedArea,
            double lensHeightInches,
            double mountAngleDegrees
    ) {
        double heightDelta = Pollen.RADIUS - lensHeightInches;
        double angleRadians = MathUtils.units.toRadians(mountAngleDegrees + targetYDegrees);
        double tangent = Math.tan(angleRadians);

        if (Math.abs(heightDelta) > MathUtils.EPSILON && Math.abs(tangent) > MathUtils.EPSILON) {
            double tyDistance = heightDelta / tangent;
            if (Double.isFinite(tyDistance) && tyDistance > 0) {
                return tyDistance;
            }
        }

        return AREA_DISTANCE_SCALE / Math.sqrt(Math.max(normalizedArea, MathUtils.EPSILON));
    }

    private static Pose robotRelativeToField(Pose robotPose, double forwardDistance, double lateralOffset) {
        return MathUtils.offsetPoseInRobotFrame(
                robotPose,
                forwardDistance,
                lateralOffset,
                Constants.FTC_COORDINATE_SYSTEM
        );
    }

    private static double scoreCandidate(double confidence, double area, double lateralOffset) {
        double lateralPenalty = Math.min(Math.abs(lateralOffset) / MAX_LATERAL_OFFSET_INCHES, 1.0);
        return SCORE_CONFIDENCE_WEIGHT * confidence
                + SCORE_AREA_WEIGHT * Math.min(area, 1.0)
                - SCORE_LATERAL_WEIGHT * lateralPenalty;
    }

    static List<DetectionCandidate> selectBestSequence(
            List<DetectionCandidate> candidates,
            int desiredCount
    ) {
        if (desiredCount <= 0 || candidates.isEmpty()) {
            return new ArrayList<>();
        }

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
            double headingFromRobot = MathUtils.headingBetween(0.0, 0.0, candidate.forwardDistance, candidate.lateralOffset);
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

                    double segmentHeading = MathUtils.headingBetween(
                            previousCandidate.forwardDistance,
                            previousCandidate.lateralOffset,
                            currentCandidate.forwardDistance,
                            currentCandidate.lateralOffset
                    );
                    double turnPenalty = SCORE_TURN_WEIGHT * Math.abs(
                            MathUtils.headingDelta(arrivalHeadings[sequenceLength - 1][previous], segmentHeading)
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

    private static PathChain buildSmoothPathChain(
            Follower follower,
            Pose robotPose,
            List<DetectionCandidate> selected
    ) {
        List<Pose> waypoints = new ArrayList<>();
        waypoints.add(new Pose(robotPose.getX(), robotPose.getY()));
        for (DetectionCandidate candidate : selected) {
            waypoints.add(candidate.fieldPose);
        }

        if (waypoints.size() < 2) return null;
        if (waypoints.size() == 2) return buildAdaptiveHandlePathChain(waypoints);

        if (shouldUsePedroCurveThrough(waypoints)) {
            return buildPedroCurveThroughPathChain(follower.pathBuilder(), waypoints);
        }

        return buildAdaptiveHandlePathChain(waypoints);
    }

    static boolean shouldUsePedroCurveThrough(List<Pose> waypoints) {
        return MathUtils.maxTurnAngle(waypoints) <= CURVE_THROUGH_MAX_TURN_RADIANS
                && MathUtils.maxSpacingRatio(waypoints) <= CURVE_THROUGH_MAX_SPACING_RATIO;
    }

    static PathChain buildPedroCurveThroughPathChain(PathBuilder pathBuilder, List<Pose> waypoints) {
        Objects.requireNonNull(pathBuilder, "pathBuilder");
        Pose start = waypoints.get(0);
        Pose mirroredPreviousPoint = start.minus(waypoints.get(1).minus(start));
        Pose[] followPoints = waypoints.subList(1, waypoints.size()).toArray(new Pose[0]);

        PathChain pathChain = pathBuilder
                .curveThrough(mirroredPreviousPoint, start, PEDRO_CURVE_THROUGH_TENSION, followPoints)
                .build();

        for (int i = 0; i < pathChain.size(); i++) {
            pathChain.getPath(i).setTangentHeadingInterpolation();
        }
        pathChain.setDecelerationType(PathChain.DecelerationType.LAST_PATH);
        return pathChain;
    }

    static PathChain buildAdaptiveHandlePathChain(List<Pose> waypoints) {
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
        return pathChain;
    }

    static final class DetectionCandidate {
        final Pose fieldPose;
        final double forwardDistance;
        final double lateralOffset;
        final double qualityScore;

        DetectionCandidate(Pose fieldPose, double forwardDistance, double lateralOffset, double qualityScore) {
            this.fieldPose = fieldPose;
            this.forwardDistance = forwardDistance;
            this.lateralOffset = lateralOffset;
            this.qualityScore = qualityScore;
        }
    }

    static final class DetectionReading {
        final double confidencePercent;
        final double targetAreaPercent;
        final double targetXDegrees;
        final double targetYDegrees;

        DetectionReading(double confidencePercent, double targetAreaPercent, double targetXDegrees, double targetYDegrees) {
            this.confidencePercent = confidencePercent;
            this.targetAreaPercent = targetAreaPercent;
            this.targetXDegrees = targetXDegrees;
            this.targetYDegrees = targetYDegrees;
        }
    }
}
