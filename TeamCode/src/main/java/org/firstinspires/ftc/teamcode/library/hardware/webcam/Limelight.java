package org.firstinspires.ftc.teamcode.library.hardware.webcam;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
public class Limelight extends Mechanism {
    public static class LIMELIGHT_PARAMS implements Mechanism.Params<LIMELIGHT_PARAMS> {
        public double
                LIMELIGHT_MOUNT_ANGLE_DEGREES = 0,
                LIMELIGHT_LENS_HEIGHT_INCHES = 0,
                TAG_HEIGHT_INCHES = 0,
                LIMELIGHT_OFFSET_X_INCHES = 0,
                LIMELIGHT_OFFSET_Y_INCHES = 0;
        public String HARDWAREMAP_NAME = "limelight";
        public boolean ISBEINGUSED = true;
        public LIMELIGHT_PARAMS() {}
        private LIMELIGHT_PARAMS(LIMELIGHT_PARAMS other) {
            LIMELIGHT_MOUNT_ANGLE_DEGREES = other.LIMELIGHT_MOUNT_ANGLE_DEGREES;
            LIMELIGHT_LENS_HEIGHT_INCHES = other.LIMELIGHT_LENS_HEIGHT_INCHES;
            TAG_HEIGHT_INCHES = other.TAG_HEIGHT_INCHES;
            LIMELIGHT_OFFSET_X_INCHES = other.LIMELIGHT_OFFSET_X_INCHES;
            LIMELIGHT_OFFSET_Y_INCHES = other.LIMELIGHT_OFFSET_Y_INCHES;
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            ISBEINGUSED = other.ISBEINGUSED;
        }
        public LIMELIGHT_PARAMS limelightMountAngleDegrees(double limelightMountAngleDegrees) {
            this.LIMELIGHT_MOUNT_ANGLE_DEGREES = limelightMountAngleDegrees;
            return this;
        }
        public LIMELIGHT_PARAMS limelightLensHeightInches(double limelightLensHeightInches) {
            this.LIMELIGHT_LENS_HEIGHT_INCHES = limelightLensHeightInches;
            return this;
        }
        public LIMELIGHT_PARAMS tagHeightInches(double tagHeightInches) {
            this.TAG_HEIGHT_INCHES = tagHeightInches;
            return this;
        }
        public LIMELIGHT_PARAMS limelightOffsetXInches(double limelightOffsetXInches) {
            this.LIMELIGHT_OFFSET_X_INCHES = limelightOffsetXInches;
            return this;
        }
        public LIMELIGHT_PARAMS limelightOffsetYInches(double limelightOffsetYInches) {
            this.LIMELIGHT_OFFSET_Y_INCHES = limelightOffsetYInches;
            return this;
        }
        public LIMELIGHT_PARAMS hardwareMapName(String hardwareMapName) {
            this.HARDWAREMAP_NAME = hardwareMapName;
            return this;
        }
        public LIMELIGHT_PARAMS isBeingUsed(boolean isBeingUsed) {
            this.ISBEINGUSED = isBeingUsed;
            return this;
        }
        @Override
        public LIMELIGHT_PARAMS updateFromBot(RobotConstants bot) {
            if (!ISBEINGUSED) return this;
            HARDWAREMAP_NAME = bot.Mechanisms.contains(Limelight.class) ? HARDWAREMAP_NAME : "";
            ISBEINGUSED = bot.Mechanisms.contains(Limelight.class);
            return this;
        }
        @Override
        public LIMELIGHT_PARAMS copy() {
            return new LIMELIGHT_PARAMS(this);
        }
    }
    public LIMELIGHT_PARAMS params;
    public Command updateCommand(GpBindings GpBindings) {
        return Commands.infinite(() -> periodic(GpBindings));
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
    public Limelight(HardwareMap hwMap, RobotConstants bot) {
        super(hwMap, bot);
        this.params = resolveParams(Limelight.class, new LIMELIGHT_PARAMS());
        if (!params.ISBEINGUSED) {
            isBeingUsed = false;
            return;
        }
        limelight1 = hwMap.get(Limelight3A.class, params.HARDWAREMAP_NAME);
        isBeingUsed = true;
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
    public LLResultTypes.DetectorResult getDetectorHighestConfidence() {
        if (!isBeingUsed) return null;
        if (detectors == null || detectors.isEmpty()) return null;
        LLResultTypes.DetectorResult highest = detectors.get(0);
        for (LLResultTypes.DetectorResult detector : detectors) {
            if (detector.getConfidence() > highest.getConfidence()) {
                highest = detector;
            }
        }
        return highest;
    }
    public void setPipeline(int pipeline) {
        limelight1.pipelineSwitch(pipeline);
    }
    public void setPipeline(Pipeline pipeline) {
        setPipeline(pipeline.getIndex());
    }
    /**
     * Enter follower.getHeading(); Must be called before LLUpdate();
     */
    public void updateRobotOrientation(double orientationRad) {
        if (!isBeingUsed) return;

        limelight1.updateRobotOrientation(
                Constants.FTC_COORDINATE_SYSTEM.convertFromPedro(
                new Pose(
                        0,
                        0,
                        orientationRad
                )
        ).getHeading() / MathUtils.DEG_TO_RAD);
    }

    public Pose getBotposeMT2() {
        if (!isBeingUsed) return null;
        if (result == null || botpose == null) return null;
        return MathUtils.units.toPedroPathing(botpose);
    }

    @Override
    public void periodic(GpBindings GpBindings) {
        LLUpdate();
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
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiduciary : fiducials) {
                    tid = fiduciary.getFiducialId();
                    dist = calculateDistance(fiduciary);
                }
            }
            botpose = result.getBotpose_MT2();
            isDisconnected = false;
            disconnectedTimer.resetTimer();
        } else {
            if (disconnectedTimer.getElapsedTimeSeconds() > 0.6) {isDisconnected= true;}
        }
    }
}
