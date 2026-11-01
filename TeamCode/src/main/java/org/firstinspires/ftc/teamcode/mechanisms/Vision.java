package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.localization.CamTransform;
import org.firstinspires.ftc.teamcode.util.localization.VisionLocalizationMath;
import org.firstinspires.ftc.teamcode.util.game.BiobuzzAprilTags;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.Collections;
import java.util.List;

@Config
public class Vision {
    public static String LIMELIGHT_NAME = "limelight";
    public static int RED_PIPELINE_INDEX = 0;
    public static int BLUE_PIPELINE_INDEX = 1;
    public static boolean USE_MEGATAG2_ORIENTATION = false;

    public static int MIN_TAGS_FOR_LOCALIZATION = 1;
    public static double MAX_RESULT_STALENESS_MS = 300;
    public static double MAX_LOCALIZATION_AMBIGUITY_M = 2.0;

    public static boolean USE_HIVE_CORRECTED_POSE = true;
    public static double FIELD_X_OFFSET_IN = -72.0;
    public static double FIELD_Y_OFFSET_IN = -72.0;
    public static double FIELD_HEADING_OFFSET_DEG = -90.0;
    public static boolean INVERT_POSE_X = false;
    public static boolean INVERT_POSE_Y = false;
    public static boolean INVERT_POSE_HEADING = false;

    public static boolean AUTO_UPDATE_FIELD_HIVE_STATE = true;
    public static double MIN_HIVE_TAG_AREA = 0.0;
    public static double HIVE_CLUSTER_AREA_MARGIN = 0.05;

    public Limelight3A limelight3A;
    public final CamTransform camTransform;
    public final BiobuzzAprilTags tags;
    public final AllianceColor allianceColor;

    private double lastTxAvgIndicator;
    private LLResult latestResult;
    private Pose2d fieldPoseEstimate = Pose2d.ZERO;
    private Field.HiveState inferredHiveState;
    private String localizationReliabilityReason = "Vision update has not run yet.";

    public Vision(HardwareMap hardwareMap, AllianceColor allianceColor) {
        if (allianceColor == null || allianceColor.isUnknown()) {
            throw new IllegalArgumentException("Vision requires alliance color RED or BLUE.");
        }
        try {
            limelight3A = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        } catch (RuntimeException runtimeException) {
            throw new IllegalStateException(
                    "Vision requires Limelight3A named '" + LIMELIGHT_NAME + "' in Robot Configuration.",
                    runtimeException
            );
        }
        camTransform = new CamTransform(limelight3A, allianceColor);
        tags = new BiobuzzAprilTags(allianceColor);
        this.allianceColor = allianceColor;
    }

    // USUAL

    public void start() {
        limelight3A.start();
        if (!limelight3A.pipelineSwitch(getDesiredPipelineIndex())) {
            throw new IllegalStateException("Unable to switch Limelight pipeline to alliance pipeline.");
        }
    }

    public void stop() {
        limelight3A.stop();
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public void update() {
        update(0.0);
    }
    public void update(double robotHeadingRadians) {
        if (USE_MEGATAG2_ORIENTATION) {
            limelight3A.updateRobotOrientation(AngleUnit.DEGREES.fromRadians(robotHeadingRadians));
        }
        camTransform.update();
        latestResult = limelight3A.getLatestResult();
        inferredHiveState = inferFieldHiveStateFromTags(inferredHiveState);
        refreshFieldPoseEstimate();
        localizationReliabilityReason = evaluateLocalizationReliability();
    }

    public Pose2d getFieldPoseEstimate() {
        return fieldPoseEstimate;
    }

    public boolean hasReliableLocalization() {
        return "OK".equals(localizationReliabilityReason);
    }

    public String getLocalizationReliabilityReason() {
        return localizationReliabilityReason;
    }

    public Field.HiveState getInferredHiveState() {
        return inferredHiveState;
    }

    public void applyInferredHiveState(Field field) {
        if (field == null) {
            throw new IllegalArgumentException("Field is required to apply inferred hive state.");
        }
        if (AUTO_UPDATE_FIELD_HIVE_STATE && inferredHiveState != null) {
            field.setHiveState(inferredHiveState);
        }
    }

    public int getDesiredPipelineIndex() {
        int pipeline = allianceColor.isBlue() ? BLUE_PIPELINE_INDEX : RED_PIPELINE_INDEX;
        if (pipeline < 0) {
            throw new IllegalStateException("Limelight pipeline index must be nonnegative.");
        }
        return pipeline;
    }

    public int getActivePipelineIndex() {
        return latestResult == null ? -1 : latestResult.getPipelineIndex();
    }

    public String getDebugString() {
        return "Pipeline=" + getActivePipelineIndex()
                + " DesiredPipeline=" + getDesiredPipelineIndex()
                + " TagCount=" + camTransform.getTagCount()
                + " Ambiguity(m)=" + camTransform.getAmbiguity()
                + " Staleness(ms)=" + (latestResult == null ? -1 : latestResult.getStaleness())
                + " Hive=" + inferredHiveState
                + " Reliability=" + localizationReliabilityReason
                + " FieldPose=" + fieldPoseEstimate;
    }

    // RELOCALIZATION

    public Pose2d getBotposeMt1() {
        return getFieldPoseEstimate();
    }
    public Pose3D getBotposeRaw() {
        return camTransform.getBotpose();
    }

    // AUTOMATED INTAKE DETECTION AND FOLLOWING


    // SHOOTER

    public double getDistance() {
        Field.HiveState hiveState = inferredHiveState == null ? Field.HiveState.UNTIPPED : inferredHiveState;
        LLResultTypes.FiducialResult bestTag = getBestCenterTag(hiveState);
        if (bestTag == null || bestTag.getTargetPoseRobotSpace() == null) {
            return -1;
        }
        Pose3D tagPoseRobotSpace = bestTag.getTargetPoseRobotSpace();
        double x = tagPoseRobotSpace.getPosition().unit.toInches(tagPoseRobotSpace.getPosition().x);
        double y = tagPoseRobotSpace.getPosition().unit.toInches(tagPoseRobotSpace.getPosition().y);
        return Math.hypot(x, y);
    }

    private int[] getCenterTagIds(Field.HiveState hiveState) {
        return getCluster(hiveState).getMidTwoTags();
    }

    private BiobuzzAprilTags.Cluster getCluster(Field.HiveState hiveState) {
        return hiveState.ordinal() + allianceColor.ordinal() == 1
                ? tags.alliance
                : tags.opposite;
    }

    public LLResultTypes.FiducialResult getBestCenterTag(
            Field.HiveState hiveState
    ) {
        LLResult result = latestResult;

        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        int[] centerTagIds = getCenterTagIds(hiveState);

        LLResultTypes.FiducialResult best = null;
        double bestDistanceSquared = Double.POSITIVE_INFINITY;

        for (LLResultTypes.FiducialResult tag : fiducials) {

            int id = tag.getFiducialId();

            if (id != centerTagIds[0] && id != centerTagIds[1]) {
                continue;
            }

            Pose3D pose = tag.getTargetPoseCameraSpace();

            if (pose == null) {
                continue;
            }

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            double distanceSquared = x * x + y * y + z * z;

            if (Double.isFinite(distanceSquared)
                    && distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                best = tag;
            }
        }

        return best;
    }

    public double getBestTx(Field.HiveState hiveState) {
        LLResultTypes.FiducialResult best = getBestCenterTag(hiveState);
        if (best == null) {
            return Double.NaN;
        }
        return best.getTargetXDegrees();
    }

    public boolean isLineupWithinTolerance(Field.HiveState hiveState, double txToleranceDeg) {
        double tx = getBestTx(hiveState);
        return Double.isFinite(tx) && Math.abs(tx) <= txToleranceDeg;
    }

    public double getAveragedTx(Field.HiveState hiveState) {
        LLResult result = latestResult;

        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            int[] tids = getCluster(hiveState).getTags();
            lastTxAvgIndicator =  result.getFiducialResults().stream()
                    .filter(tag -> tag.getTargetArea()>=0 && (tag.getFiducialId() == tids[0] || tag.getFiducialId() == tids[1] || tag.getFiducialId() == tids[2] || tag.getFiducialId() == tids[3]))
                    .mapToDouble(LLResultTypes.FiducialResult::getTargetXDegrees)
                    .average().orElse(lastTxAvgIndicator);
        }
        return lastTxAvgIndicator;
    }


    public List<LLResultTypes.FiducialResult> fiducials() {
        if (latestResult == null || latestResult.getFiducialResults() == null) {
            return Collections.emptyList();
        }
        return latestResult.getFiducialResults();
    }

    private Field.HiveState inferFieldHiveStateFromTags(Field.HiveState previousState) {
        if (latestResult == null || !latestResult.isValid() || latestResult.getFiducialResults() == null) {
            return null;
        }
        double allianceArea = 0.0;
        double oppositeArea = 0.0;
        for (LLResultTypes.FiducialResult fiducial : latestResult.getFiducialResults()) {
            double area = fiducial.getTargetArea();
            if (!Double.isFinite(area) || area < MIN_HIVE_TAG_AREA) {
                continue;
            }
            int id = fiducial.getFiducialId();
            if (contains(tags.alliance, id)) {
                allianceArea += area;
            }
            if (contains(tags.opposite, id)) {
                oppositeArea += area;
            }
        }
        if (allianceArea <= 0 && oppositeArea <= 0) {
            return null;
        }
        double diff = Math.abs(allianceArea - oppositeArea);
        if (diff < HIVE_CLUSTER_AREA_MARGIN && previousState != null) {
            return previousState;
        }
        boolean allianceClusterDetected = allianceArea >= oppositeArea;
        return VisionLocalizationMath.clusterToFieldHiveState(allianceClusterDetected, allianceColor);
    }

    private boolean contains(BiobuzzAprilTags.Cluster cluster, int fiducialId) {
        int[] tags = cluster.getTags();
        for (int tag : tags) {
            if (tag == fiducialId) {
                return true;
            }
        }
        return false;
    }

    private void refreshFieldPoseEstimate() {
        if (!camTransform.isValid()) {
            return;
        }
        Pose2d limelightPose = USE_HIVE_CORRECTED_POSE ? camTransform.getPose() : camTransform.getRawPose();
        fieldPoseEstimate = VisionLocalizationMath.toFieldPose(
                limelightPose,
                FIELD_X_OFFSET_IN,
                FIELD_Y_OFFSET_IN,
                FIELD_HEADING_OFFSET_DEG,
                INVERT_POSE_X,
                INVERT_POSE_Y,
                INVERT_POSE_HEADING
        );
    }

    private String evaluateLocalizationReliability() {
        if (latestResult == null) {
            return "No Limelight frame available.";
        }
        if (!latestResult.isValid() || !camTransform.isValid()) {
            return "Limelight frame is invalid.";
        }
        if (latestResult.getStaleness() > MAX_RESULT_STALENESS_MS) {
            return "Limelight frame is stale (" + latestResult.getStaleness() + "ms).";
        }
        if (camTransform.getTagCount() < MIN_TAGS_FOR_LOCALIZATION) {
            return "Not enough tags for localization (" + camTransform.getTagCount() + ").";
        }
        if (MAX_LOCALIZATION_AMBIGUITY_M > 0
                && Double.isFinite(MAX_LOCALIZATION_AMBIGUITY_M)
                && camTransform.getAmbiguity() > MAX_LOCALIZATION_AMBIGUITY_M) {
            return "Pose ambiguity too high (" + camTransform.getAmbiguity() + ").";
        }
        return "OK";
    }

}
