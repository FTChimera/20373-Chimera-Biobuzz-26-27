package org.firstinspires.ftc.teamcode.library.hardware.webcam;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
public class Limelight {
    public static class LIMELIGHT_PARAMS {
        public final double
                LIMELIGHT_MOUNT_ANGLE_DEGREES = 0,
                LIMELIGHT_LENS_HEIGHT_INCHES = 0,
                TAG_HEIGHT_INCHES = 0;
        public final String HARDWAREMAP_NAME = "limelight";
    }
    public LIMELIGHT_PARAMS params;
    public Command updateCommand() {
        return Commands.infinite(Limelight.this::LLUpdate);
    }

    public enum Pipeline {
        APRIL_TAG(0),
        COLOR(1),
        NEURAL_DETECTOR(2);
        private final int pipelineIndex;
        Pipeline(int index) { this.pipelineIndex = index; }
        public static final Pipeline defaultPipeline = APRIL_TAG;
        public int getIndex() { return pipelineIndex; }
    }
    private final boolean isBeingUsed;
    public Limelight(HardwareMap hwMap, LIMELIGHT_PARAMS params) {
        limelight1 = hwMap.get(Limelight3A.class, params.HARDWAREMAP_NAME);
        this.params = params;
        isBeingUsed = true;
    }
    public Limelight() {
        isBeingUsed = false;
    }
    public double tx=0,ty=0,ta=0,tid=0, dist=0;
    public boolean isDisconnected;
    public Limelight3A limelight1;
    private Timer disconnectedTimer;
    public Pose3D botpose;
    public LLResult result;
    private List<LLResultTypes.FiducialResult> fiducials;
    private List<LLResultTypes.ColorResult> colors;
    private List<LLResultTypes.DetectorResult> detectors; // From Neural Networks
    public void start(int pipeline) {startLLWithPipeline(pipeline);}
    public void start() {start(Pipeline.defaultPipeline);}
    public void start(Pipeline pipeline) {
        start(pipeline.getIndex());
    }
    private void startLLWithPipeline(int pipeline){
        if (!isBeingUsed) return;
        limelight1.start();limelight1.pipelineSwitch(pipeline);
        disconnectedTimer = new Timer();
    }
    public void stop() {
        if (!isBeingUsed) return;
        limelight1.stop();
    }
    public LLResultTypes.FiducialResult getResultForTag(int tagID) {
        if (!isBeingUsed) return null;
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
            if (fiduciary.getFiducialId() == tagID) {
                return fiduciary;
            }
        }
        return null;
    }
    public double calculateDistance(LLResultTypes.FiducialResult result) {
        double ty = result.getTargetYDegrees();

        double ANGLE_TO_TAG_RADIANS = Math.toRadians(params.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
        return (params.TAG_HEIGHT_INCHES - params.LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(ANGLE_TO_TAG_RADIANS);
    }
    public double findDistanceFromTag(int tagId) {
        if (!isBeingUsed) return Double.NaN;
        LLResultTypes.FiducialResult fiduciary = getResultForTag(tagId);
        if (fiduciary == null) return Double.NaN;
        return calculateDistance(fiduciary);
    }

    public boolean isTagInFiducialResults(int tagID) {
        if (!isBeingUsed) return false;
        if (fiducials == null) return false;
        for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
            if (fiduciary.getFiducialId() == tagID) {
                return true;
            }
        }
        return false;
    }
    public List<LLResultTypes.FiducialResult> getFiducials() {
        if (!isBeingUsed) return null;
        return fiducials;
    }
    public List<LLResultTypes.ColorResult> getColors() {
        if (!isBeingUsed) return null;
        return colors;
    }
    public List<LLResultTypes.DetectorResult> getDetectors() {
        if (!isBeingUsed) return null;
        return detectors;
    }
    public void updateRobotOrientation(double orientationDegrees) {
        if (!isBeingUsed) return;
        limelight1.updateRobotOrientation(orientationDegrees);
    }
    public void LLUpdate() {
        // Must be called every loop for disconnected timer
        if (!isBeingUsed) return;
        result = limelight1.getLatestResult();
        if (result!=null && result.isValid()) {
            tx=result.getTx();
            ty= result.getTy();
            ta=result.getTa();
            fiducials = result.getFiducialResults();
            colors = result.getColorResults();
            detectors = result.getDetectorResults();
            for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
                tid = fiduciary.getFiducialId();
                dist = calculateDistance(fiduciary);
            }
            botpose = result.getBotpose_MT2();
            isDisconnected = false;
            disconnectedTimer.resetTimer();
        } else {
            if (disconnectedTimer.getElapsedTimeSeconds() > 0.6) {isDisconnected= true;}
        }
    }
    public double getRGBScore() {
        if (!isBeingUsed) return Double.NaN;
        return Math.abs(tx);
    }

}

