package org.firstinspires.ftc.teamcode.library;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class IntakePallPath {
    public static final double DETECTOR_TARGET_HEIGHT_INCHES = 1.5;
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

    public static PathChain generateIntakePathFromCam() {
        return new PathChain();
    }
}
