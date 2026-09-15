package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.library.IntakeBallPath;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.List;
import java.util.Objects;

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
        public Button AUTO_INTAKE_BUTTON = Button.DPAD_DOWN;
        public VISION_PARAMS limelightParams(Limelight.LIMELIGHT_PARAMS params) {
            LIMELIGHT_PARAMS = params.copy();
            return this;
        }
        public VISION_PARAMS autoIntakeButton(Button button) {
            AUTO_INTAKE_BUTTON = button.copy();
            return this;
        }

        @Override
        public VISION_PARAMS copy() {
            return new VISION_PARAMS().limelightParams(LIMELIGHT_PARAMS).autoIntakeButton(AUTO_INTAKE_BUTTON);
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

    @Override
    public void stop() {
        camera.stop();
    }

    public void setPipeline(Pipeline pipeline) {
        camera.setPipeline(pipeline.cameraPipeline);
    }
    public void setPipeline(int pipeline) {
        camera.setPipeline(pipeline);
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

    public boolean isRunning() {
        return camera.isRunning();
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
        return getBotposeMT2();
    }

    /**
     * Updates the Limelight result and applies its MT2 pose only when that result is valid.
     *
     * @return whether a vision pose was applied to the drive localizer
     */
    public boolean forceRelocalize(RobotDrive drive) {
        double CAMERA_CONFIDENCE = 0.3;//0 to 1
        Objects.requireNonNull(drive, "drive");
        if (!isRunning()) {
            return false;
        }
        if (!hasValidResult()) {
            return false;
        }
        int pipeline = getLatestResult().getPipelineIndex();
        setPipeline(Pipeline.APRIL_TAG);
        updateRobotOrientation(drive.getPose().getHeading());
        update();
        Pose visionPose = getCachedPose();
        // A simple low-pass filter
        drive.setPose(visionPose.linearCombination(
                drive.getPose(),
                CAMERA_CONFIDENCE,1-CAMERA_CONFIDENCE
        ));
        setPipeline(pipeline);
        return true;
    }

    @Override
    public void periodic(State state, RobotMode robotMode) {
        //forceRelocalize(state.getRobot().getDrive());
        updateRobotOrientation(state.getRobot().getPose().getHeading());
        update();

        if (robotMode==RobotMode.TELEOP) {
            // now some gamepad commands
            // some command to follow the generated path to pick up balls
            // this is if the driver needs to focus on something else
            // todo move that away from Utils in pathplanner to here so that we can call it in teleop
            if (state.getGpBindings().getCommand(params.AUTO_INTAKE_BUTTON).wasJustPressed()) {
                PathChain intakeChain = IntakeBallPath.generateIntakePathFromCam();
                state.getRobot().getDrive().followPath(intakeChain);
            }
        }
    }
}
