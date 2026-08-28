package org.firstinspires.ftc.teamcode.library.hardware.webcam;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;

import java.util.List;
import java.util.Objects;

/**
 * Standalone controller for a Limelight 3A. Call {@link #update()} every loop while it is active.
 */
@SuppressWarnings("SpellCheckingInspection")
public class Limelight {
    public static class LIMELIGHT_PARAMS {
        public double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0;
        public double LIMELIGHT_LENS_HEIGHT_INCHES = 0;
        public double TAG_HEIGHT_INCHES = 0;
        public double LIMELIGHT_OFFSET_X_INCHES = 0;
        public double LIMELIGHT_OFFSET_Y_INCHES = 0;
        public String HARDWAREMAP_NAME = "limelight";
        public int POLL_RATE_HZ = 100;
        public double DISCONNECT_TIMEOUT_SECONDS = 0.6;

        public LIMELIGHT_PARAMS() {}

        private LIMELIGHT_PARAMS(LIMELIGHT_PARAMS other) {
            LIMELIGHT_MOUNT_ANGLE_DEGREES = other.LIMELIGHT_MOUNT_ANGLE_DEGREES;
            LIMELIGHT_LENS_HEIGHT_INCHES = other.LIMELIGHT_LENS_HEIGHT_INCHES;
            TAG_HEIGHT_INCHES = other.TAG_HEIGHT_INCHES;
            LIMELIGHT_OFFSET_X_INCHES = other.LIMELIGHT_OFFSET_X_INCHES;
            LIMELIGHT_OFFSET_Y_INCHES = other.LIMELIGHT_OFFSET_Y_INCHES;
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            POLL_RATE_HZ = other.POLL_RATE_HZ;
            DISCONNECT_TIMEOUT_SECONDS = other.DISCONNECT_TIMEOUT_SECONDS;
        }

        public LIMELIGHT_PARAMS limelightMountAngleDegrees(double value) {
            LIMELIGHT_MOUNT_ANGLE_DEGREES = value;
            return this;
        }

        public LIMELIGHT_PARAMS limelightLensHeightInches(double value) {
            LIMELIGHT_LENS_HEIGHT_INCHES = value;
            return this;
        }

        public LIMELIGHT_PARAMS tagHeightInches(double value) {
            TAG_HEIGHT_INCHES = value;
            return this;
        }

        public LIMELIGHT_PARAMS limelightOffsetXInches(double value) {
            LIMELIGHT_OFFSET_X_INCHES = value;
            return this;
        }

        public LIMELIGHT_PARAMS limelightOffsetYInches(double value) {
            LIMELIGHT_OFFSET_Y_INCHES = value;
            return this;
        }

        public LIMELIGHT_PARAMS hardwareMapName(String value) {
            HARDWAREMAP_NAME = Objects.requireNonNull(value, "hardwareMapName");
            return this;
        }

        public LIMELIGHT_PARAMS pollRateHz(int value) {
            if (value < 1 || value > 250) {
                throw new IllegalArgumentException("pollRateHz must be between 1 and 250.");
            }
            POLL_RATE_HZ = value;
            return this;
        }

        public LIMELIGHT_PARAMS disconnectTimeoutSeconds(double value) {
            if (value < 0) {
                throw new IllegalArgumentException("disconnectTimeoutSeconds must not be negative.");
            }
            DISCONNECT_TIMEOUT_SECONDS = value;
            return this;
        }

        public LIMELIGHT_PARAMS copy() {
            return new LIMELIGHT_PARAMS(this);
        }
    }

    public enum Pipeline {
        APRIL_TAG(0),
        COLOR(1),
        NEURAL_DETECTOR(2);

        private final int pipelineIndex;

        Pipeline(int index) {
            pipelineIndex = index;
        }

        public static final Pipeline defaultPipeline = APRIL_TAG;

        public int getIndex() {
            return pipelineIndex;
        }
    }

    public final LIMELIGHT_PARAMS params;
    private final Limelight3A device;
    private long lastValidResultNanos;

    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public double tid = 0;
    public double dist = 0;
    public boolean isDisconnected = true;
    public Pose3D botpose;
    public LLResult result;
    private List<LLResultTypes.FiducialResult> fiducials;
    private List<LLResultTypes.ColorResult> colors;
    private List<LLResultTypes.DetectorResult> detectors;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, new LIMELIGHT_PARAMS());
    }

    public Limelight(HardwareMap hardwareMap, LIMELIGHT_PARAMS params) {
        this(hardwareMap.get(Limelight3A.class, requireHardwareMapName(params)), params);
    }

    public Limelight(Limelight3A device, LIMELIGHT_PARAMS params) {
        this.device = Objects.requireNonNull(device, "device");
        this.params = Objects.requireNonNull(params, "params").copy();
    }

    public Limelight3A getDevice() {
        return device;
    }

    public void start() {
        start(Pipeline.defaultPipeline);
    }

    public void start(Pipeline pipeline) {
        start(pipeline.getIndex());
    }

    public int getPipelineIndex() {
        LLResult latestResult = device.getLatestResult();
        if (latestResult == null) {
            throw new IllegalStateException("Limelight has not produced a result yet.");
        }
        return latestResult.getPipelineIndex();
    }

    public void start(int pipeline) {
        setPipeline(pipeline);
        device.setPollRateHz(params.POLL_RATE_HZ);
        device.start();
        lastValidResultNanos = System.nanoTime();
        isDisconnected = false;
    }

    public void stop() {
        device.stop();
        isDisconnected = true;
    }

    public void pause() {
        device.pause();
    }

    public boolean isRunning() {
        return device.isRunning();
    }

    public boolean setPipeline(Pipeline pipeline) {
        return setPipeline(pipeline.getIndex());
    }

    public boolean setPipeline(int pipeline) {
        if (pipeline < 0) {
            throw new IllegalArgumentException("pipeline must not be negative.");
        }
        return device.pipelineSwitch(pipeline);
    }

    public boolean reloadPipeline() {
        return device.reloadPipeline();
    }

    public boolean captureSnapshot(String name) {
        return device.captureSnapshot(Objects.requireNonNull(name, "name"));
    }

    public boolean deleteSnapshot(String name) {
        return device.deleteSnapshot(Objects.requireNonNull(name, "name"));
    }

    public boolean deleteSnapshots() {
        return device.deleteSnapshots();
    }

    public boolean updatePythonInputs(double... inputs) {
        return device.updatePythonInputs(Objects.requireNonNull(inputs, "inputs"));
    }

    /** Updates the robot heading in Limelight's FTC coordinate frame. */
    public boolean updateRobotOrientation(double orientationRad) {
        return device.updateRobotOrientation(
                Constants.FTC_COORDINATE_SYSTEM.convertFromPedro(
                        new Pose(0, 0, orientationRad)
                ).getHeading() / MathUtils.DEG_TO_RAD
        );
    }

    public LLResultTypes.FiducialResult getResultForTag(int tagId) {
        if (fiducials == null) {
            return null;
        }
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == tagId) {
                return fiducial;
            }
        }
        return null;
    }

    public boolean isTagInFiducialResults(int tagId) {
        return getResultForTag(tagId) != null;
    }

    public double calculateDistance(LLResultTypes.FiducialResult fiducial) {
        Objects.requireNonNull(fiducial, "fiducial");
        double angleToTagRadians = Math.toRadians(
                params.LIMELIGHT_MOUNT_ANGLE_DEGREES + fiducial.getTargetYDegrees()
        );
        return (params.TAG_HEIGHT_INCHES - params.LIMELIGHT_LENS_HEIGHT_INCHES)
                / Math.tan(angleToTagRadians);
    }

    public double findDistanceFromTag(int tagId) {
        LLResultTypes.FiducialResult fiducial = getResultForTag(tagId);
        return fiducial == null ? Double.NaN : calculateDistance(fiducial);
    }

    public List<LLResultTypes.FiducialResult> getFiducials() {
        return fiducials;
    }

    public List<LLResultTypes.ColorResult> getColors() {
        return colors;
    }

    public List<LLResultTypes.DetectorResult> getDetectors() {
        return detectors;
    }

    public LLResultTypes.DetectorResult getDetectorHighestConfidence() {
        if (detectors == null || detectors.isEmpty()) {
            return null;
        }
        LLResultTypes.DetectorResult highest = detectors.get(0);
        for (LLResultTypes.DetectorResult detector : detectors) {
            if (detector.getConfidence() > highest.getConfidence()) {
                highest = detector;
            }
        }
        return highest;
    }

    public Pose getBotposeMT2() {
        return result == null || botpose == null ? null : MathUtils.units.toPedroPathing(botpose);
    }

    public void LLUpdate() {
        update();
    }

    /** Caches the latest Limelight results and refreshes connection state. */
    public void update() {
        result = device.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            fiducials = result.getFiducialResults();
            colors = result.getColorResults();
            detectors = result.getDetectorResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    tid = fiducial.getFiducialId();
                    dist = calculateDistance(fiducial);
                }
            }
            botpose = result.getBotpose_MT2();
            lastValidResultNanos = System.nanoTime();
            isDisconnected = false;
            return;
        }
        isDisconnected = !device.isRunning()
                || elapsedSecondsSince(lastValidResultNanos) > params.DISCONNECT_TIMEOUT_SECONDS;
    }

    private static String requireHardwareMapName(LIMELIGHT_PARAMS params) {
        String name = Objects.requireNonNull(params, "params").HARDWAREMAP_NAME;
        if (name.isBlank()) {
            throw new IllegalArgumentException("hardwareMapName must not be blank.");
        }
        return name;
    }

    private static double elapsedSecondsSince(long startNanos) {
        return (System.nanoTime() - startNanos) / 1_000_000_000.0;
    }
}
