package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.rmsftc.utils.AllianceColor;
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
    public static int BLUE_PIPELINE_INDEX = 0;
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
    private final FrameFreshness frameFreshness = new FrameFreshness();
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
        if (latestResult == null) {
            frameFreshness.invalidate();
        } else {
            // Read wall-clock staleness after sampling the monotonic clock, conservatively.
            long observedNanos = System.nanoTime();
            frameFreshness.observe(
                    latestResult.isValid(), latestResult.getPipelineIndex(), latestResult.getTimestamp(),
                    latestResult.getStaleness(), latestResult.getCaptureLatency(),
                    latestResult.getTargetingLatency(), latestResult.getParseLatency(), observedNanos
            );
        }
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
    public boolean anyFiducial() {
        return !fiducials().isEmpty();
    }

    /**
     * True when a valid, current-pipeline frame within {@link #MAX_RESULT_STALENESS_MS}
     * of estimated capture contains any AprilTag from OUR alliance's hive. The hive
     * carries two tag clusters (one per face), and which face the camera
     * sees depends on whether the hive is tipped, so both clusters count.
     * Tags from the other alliance's hive do not count: the far hive is
     * visible from some shoot poses and must not end the lineup wait.
     */
    public boolean anyAllianceFiducial() {
        if (!frameFreshness.isFresh(
                getDesiredPipelineIndex(), System.nanoTime(), MAX_RESULT_STALENESS_MS)) {
            return false;
        }
        return containsAllianceFiducial();
    }

    /**
     * As {@link #anyAllianceFiducial()}, additionally requiring estimated capture strictly
     * after {@code notBeforeNanos}, sampled with {@link System#nanoTime()} at settle end.
     * Call {@link #update()} as usual before querying; this method does not poll the camera.
     *
     * Capture is conservatively estimated at first observation minus SDK staleness,
     * capture, targeting and parse latencies, plus a 1 ms wall-clock quantization guard.
     * Repeated camera timestamps never advance that estimate. Unreported network latency
     * cannot be recovered from the SDK, so this is not a synchronized exposure timestamp.
     */
    public boolean anyAllianceFiducialAfter(long notBeforeNanos) {
        if (!frameFreshness.isFreshAfter(
                getDesiredPipelineIndex(), System.nanoTime(), MAX_RESULT_STALENESS_MS,
                notBeforeNanos)) {
            return false;
        }
        return containsAllianceFiducial();
    }

    private boolean containsAllianceFiducial() {
        for (LLResultTypes.FiducialResult fiducialResult : fiducials()) {
            int id = fiducialResult.getFiducialId();
            if (contains(tags.alliance, id) || contains(tags.opposite, id)) {
                return true;
            }
        }
        return false;
    }

    /** Monotonic freshness bookkeeping, kept independent of SDK hardware for unit tests. */
    static final class FrameFreshness {
        private boolean observed;
        private boolean valid;
        private int pipeline;
        private double timestamp;
        private long observedNanos;
        private double captureAgeMs;

        void invalidate() {
            valid = false;
        }

        void observe(boolean frameValid, int framePipeline, double frameTimestamp,
                     long stalenessMs, double captureMs, double targetingMs, double parseMs,
                     long nowNanos) {
            valid = false;
            if (!frameValid || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0
                    || stalenessMs < 0 || !nonnegativeFinite(captureMs)
                    || !nonnegativeFinite(targetingMs) || !nonnegativeFinite(parseMs)) {
                return;
            }
            double ageMs = stalenessMs + captureMs + targetingMs + parseMs + 1.0;
            if (!Double.isFinite(ageMs)) {
                return;
            }
            if (!observed || frameTimestamp != timestamp) {
                observed = true;
                timestamp = frameTimestamp;
                observedNanos = nowNanos;
                captureAgeMs = ageMs;
            } else {
                // Later receipts may make the estimate older, never newer.
                captureAgeMs = Math.max(captureAgeMs, ageMs - (nowNanos - observedNanos) / 1e6);
            }
            pipeline = framePipeline;
            valid = true;
        }

        boolean isFresh(int desiredPipeline, long nowNanos, double maxAgeMs) {
            long elapsedNanos = nowNanos - observedNanos;
            return valid && pipeline == desiredPipeline && nonnegativeFinite(maxAgeMs)
                    && elapsedNanos >= 0
                    && captureAgeMs + elapsedNanos / 1e6 <= maxAgeMs;
        }

        boolean isFreshAfter(int desiredPipeline, long nowNanos, double maxAgeMs,
                             long notBeforeNanos) {
            return isFresh(desiredPipeline, nowNanos, maxAgeMs)
                    && (observedNanos - notBeforeNanos) / 1e6 > captureAgeMs;
        }

        private static boolean nonnegativeFinite(double value) {
            return Double.isFinite(value) && value >= 0;
        }
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
        return "Alliance=" + allianceColor
                + " Pipeline=" + getActivePipelineIndex()
                + " DesiredPipeline=" + getDesiredPipelineIndex()
                + " TagCount=" + camTransform.getTagCount()
                + " VisibleTagIds=" + visibleTagIds()
                + " AnyAllianceTag=" + anyAllianceFiducial()
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


    /** Comma-separated IDs of every fiducial in the latest frame, for pit debugging. */
    public String visibleTagIds() {
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult fiducial : fiducials()) {
            if (ids.length() > 0) ids.append(",");
            ids.append(fiducial.getFiducialId());
        }
        return ids.length() == 0 ? "none" : ids.toString();
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

    public static StubVision stub(HardwareMap hardwareMap, AllianceColor allianceColor) {
        return new StubVision(hardwareMap, allianceColor);
    }

    public static class StubVision extends Vision {
        public StubVision(HardwareMap hardwareMap, AllianceColor allianceColor) {
            super(hardwareMap, allianceColor);
        }

        @Override
        public void start() {}

        @Override
        public void stop() {}

        @Override
        public LLResult getLatestResult() { return null; }

        @Override
        public void update(double robotHeadingRadians) {}

        @Override
        public boolean hasReliableLocalization() {return false;}


    }
}
