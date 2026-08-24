package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.List;

public class Vision extends Mechanism {
    private final Limelight camera;
    public final VISION_PARAMS params;

    public enum Pipeline {
        APRIL_TAG(Limelight.Pipeline.APRIL_TAG),
        COLOR(Limelight.Pipeline.COLOR),
        NEURAL_DETECTOR(Limelight.Pipeline.NEURAL_DETECTOR);

        private final Limelight.Pipeline cameraPipeline;

        Pipeline(Limelight.Pipeline cameraPipeline) {
            this.cameraPipeline = cameraPipeline;
        }
    }

    public static class VISION_PARAMS implements Mechanism.Params<VISION_PARAMS> {
        public Limelight.LIMELIGHT_PARAMS LIMELIGHT_PARAMS = new Limelight.LIMELIGHT_PARAMS();

        public VISION_PARAMS limelightParams(Limelight.LIMELIGHT_PARAMS params) {
            LIMELIGHT_PARAMS = params.copy();
            return this;
        }

        @Override
        public VISION_PARAMS copy() {
            return new VISION_PARAMS().limelightParams(LIMELIGHT_PARAMS);
        }

        @Override
        public VISION_PARAMS updateFromBot(RobotConstants bot) {
            return this;
        }
    }

    public Vision(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(Vision.class, new VISION_PARAMS());
        camera = new Limelight(hardwareMap, params.LIMELIGHT_PARAMS);
    }

    public void start(Pipeline pipeline) {
        camera.start(pipeline.cameraPipeline);
    }

    public void stop() {
        camera.stop();
    }

    public void setPipeline(Pipeline pipeline) {
        camera.setPipeline(pipeline.cameraPipeline);
    }

    public LLResult getLatestResult() {
        update();
        return camera.result;
    }

    public boolean updateRobotOrientation(double orientationRad) {
        return camera.updateRobotOrientation(orientationRad);
    }

    public boolean hasValidResult() {
        return camera.result != null && camera.result.isValid();
    }

    public boolean isDisconnected() {
        return camera.isDisconnected;
    }

    public double getLastTagId() {
        return camera.tid;
    }

    public double getLastDistanceInches() {
        return camera.dist;
    }

    public double getLensHeightInches() {
        return params.LIMELIGHT_PARAMS.LIMELIGHT_LENS_HEIGHT_INCHES;
    }

    public double getMountAngleDegrees() {
        return params.LIMELIGHT_PARAMS.LIMELIGHT_MOUNT_ANGLE_DEGREES;
    }

    public Pose getBotposeMT2() {
        return camera.getBotposeMT2();
    }

    public List<LLResultTypes.FiducialResult> getFiducials() {
        return camera.getFiducials();
    }

    public List<LLResultTypes.ColorResult> getColors() {
        return camera.getColors();
    }

    public List<LLResultTypes.DetectorResult> getDetectors() {
        return camera.getDetectors();
    }

    public LLResultTypes.DetectorResult getDetectorHighestConfidence() {
        return camera.getDetectorHighestConfidence();
    }

    public void update() {
        camera.update();
    }

    public Pose getCachedPose() {
        return MathUtils.units.toPedroPathing(camera.botpose);
    }


    @Override
    public void periodic(GpBindings GpBindings) {
        update();
    }
}
