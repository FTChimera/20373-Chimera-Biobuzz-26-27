package org.firstinspires.ftc.teamcode.library.hardware.webcam;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

/** Standalone AprilTag controller for a Logitech or other FTC USB webcam. */
@SuppressWarnings("SpellCheckingInspection")
public class LogitechWebcam implements AutoCloseable {
    public static class LOGITECH_WEBCAM_PARAMS {
        public String HARDWAREMAP_NAME = "Webcam 1";
        public boolean SET_DRAW_AXES = true;
        public boolean SET_DRAW_CUBE_PROJECTION = true;
        public boolean SET_DRAW_TAG_ID = true;
        public boolean SET_DRAW_TAG_OUTLINE = true;
        public DistanceUnit OUTPUT_DISTANCE_UNIT = DistanceUnit.INCH;
        public AngleUnit OUTPUT_ANGLE_UNIT = AngleUnit.RADIANS;
        public int CAMERA_RESOLUTION_WIDTH = 640;
        public int CAMERA_RESOLUTION_HEIGHT = 480;

        public LOGITECH_WEBCAM_PARAMS() {}

        private LOGITECH_WEBCAM_PARAMS(LOGITECH_WEBCAM_PARAMS other) {
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            SET_DRAW_AXES = other.SET_DRAW_AXES;
            SET_DRAW_CUBE_PROJECTION = other.SET_DRAW_CUBE_PROJECTION;
            SET_DRAW_TAG_ID = other.SET_DRAW_TAG_ID;
            SET_DRAW_TAG_OUTLINE = other.SET_DRAW_TAG_OUTLINE;
            OUTPUT_DISTANCE_UNIT = other.OUTPUT_DISTANCE_UNIT;
            OUTPUT_ANGLE_UNIT = other.OUTPUT_ANGLE_UNIT;
            CAMERA_RESOLUTION_WIDTH = other.CAMERA_RESOLUTION_WIDTH;
            CAMERA_RESOLUTION_HEIGHT = other.CAMERA_RESOLUTION_HEIGHT;
        }

        public LOGITECH_WEBCAM_PARAMS hardwareMapName(String value) {
            HARDWAREMAP_NAME = Objects.requireNonNull(value, "hardwareMapName");
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS setDrawAxes(boolean value) {
            SET_DRAW_AXES = value;
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS setDrawCubeProjection(boolean value) {
            SET_DRAW_CUBE_PROJECTION = value;
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS setDrawTagId(boolean value) {
            SET_DRAW_TAG_ID = value;
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS setDrawTagOutline(boolean value) {
            SET_DRAW_TAG_OUTLINE = value;
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS outputDistanceUnit(DistanceUnit value) {
            OUTPUT_DISTANCE_UNIT = Objects.requireNonNull(value, "outputDistanceUnit");
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS outputAngleUnit(AngleUnit value) {
            OUTPUT_ANGLE_UNIT = Objects.requireNonNull(value, "outputAngleUnit");
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS cameraResolution(int width, int height) {
            if (width <= 0 || height <= 0) {
                throw new IllegalArgumentException("camera resolution must be positive.");
            }
            CAMERA_RESOLUTION_WIDTH = width;
            CAMERA_RESOLUTION_HEIGHT = height;
            return this;
        }

        public LOGITECH_WEBCAM_PARAMS copy() {
            return new LOGITECH_WEBCAM_PARAMS(this);
        }
    }

    public final LOGITECH_WEBCAM_PARAMS params;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    public AprilTagPoseFtc AprilTagPose;
    public Pose3D botpose;
    public com.pedropathing.geometry.Pose pedroPose;
    public int id;
    public boolean hasDetection;
    public boolean isDisconnected;
    private List<AprilTagDetection> currentDetections;

    public LogitechWebcam(HardwareMap hardwareMap) {
        this(hardwareMap, new LOGITECH_WEBCAM_PARAMS());
    }

    public LogitechWebcam(HardwareMap hardwareMap, LOGITECH_WEBCAM_PARAMS params) {
        this.params = Objects.requireNonNull(params, "params").copy();
        if (this.params.HARDWAREMAP_NAME.isBlank()) {
            throw new IllegalArgumentException("hardwareMapName must not be blank.");
        }
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(this.params.SET_DRAW_AXES)
                .setDrawCubeProjection(this.params.SET_DRAW_CUBE_PROJECTION)
                .setDrawTagID(this.params.SET_DRAW_TAG_ID)
                .setDrawTagOutline(this.params.SET_DRAW_TAG_OUTLINE)
                .setOutputUnits(this.params.OUTPUT_DISTANCE_UNIT, this.params.OUTPUT_ANGLE_UNIT)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, this.params.HARDWAREMAP_NAME))
                .setCameraResolution(new Size(
                        this.params.CAMERA_RESOLUTION_WIDTH,
                        this.params.CAMERA_RESOLUTION_HEIGHT
                ))
                .build();
    }

    public AprilTagProcessor getTagProcessor() {
        return tagProcessor;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    /** Polls the portal and caches the first valid AprilTag detection. */
    public void update() {
        currentDetections = tagProcessor.getDetections();
        isDisconnected = visionPortal.getCameraState() == VisionPortal.CameraState.ERROR;
        if (currentDetections == null || currentDetections.isEmpty()) {
            hasDetection = false;
            return;
        }

        AprilTagDetection tag = currentDetections.get(0);
        if (tag.ftcPose == null) {
            hasDetection = false;
            return;
        }

        AprilTagPose = tag.ftcPose;
        id = tag.id;
        Position position = new Position(
                params.OUTPUT_DISTANCE_UNIT,
                AprilTagPose.x,
                AprilTagPose.y,
                AprilTagPose.z,
                0
        );
        YawPitchRollAngles orientation = new YawPitchRollAngles(
                params.OUTPUT_ANGLE_UNIT,
                AprilTagPose.yaw,
                AprilTagPose.pitch,
                AprilTagPose.roll,
                0
        );
        botpose = new Pose3D(position, orientation);
        pedroPose = toPedro(botpose);
        hasDetection = true;
    }

    public AprilTagDetection getDetectionForTag(int tagId) {
        if (currentDetections == null) {
            return null;
        }
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    public boolean isTagVisible(int tagId) {
        return getDetectionForTag(tagId) != null;
    }

    public List<AprilTagDetection> getDetections() {
        return currentDetections;
    }

    public int getDetectionCount() {
        return currentDetections == null ? 0 : currentDetections.size();
    }

    @Override
    public void close() {
        visionPortal.close();
    }

    private static com.pedropathing.geometry.Pose toPedro(Pose3D pose3D) {
        return new com.pedropathing.geometry.Pose(
                pose3D.getPosition().toUnit(DistanceUnit.INCH).x,
                pose3D.getPosition().toUnit(DistanceUnit.INCH).z,
                pose3D.getOrientation().getYaw(AngleUnit.RADIANS)
        );
    }
}
