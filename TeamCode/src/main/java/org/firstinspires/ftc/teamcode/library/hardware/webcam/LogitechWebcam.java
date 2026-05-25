package org.firstinspires.ftc.teamcode.library.hardware.webcam;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Hardware wrapper for a Logitech (or any USB) webcam using the FTC VisionPortal AprilTag pipeline.
 * Detects AprilTags and provides the robot's field pose derived from the best visible tag.
 *
 * <pre>
 * LogitechWebcam cam = new LogitechWebcam(hardwareMap);         // default "Webcam 1"
 * LogitechWebcam cam = new LogitechWebcam(hardwareMap, "Cam2"); // custom name
 * cam.update();
 * if (cam.hasDetection) { ... cam.botpose ... }
 * </pre>
 */
@SuppressWarnings("SpellCheckingInspection")
public class LogitechWebcam {
    /** Parameters for the Logitech webcam. */
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
        public boolean ISBEINGUSED = true;
        public void updateFromBot(RobotConstants bot) {
            if (!ISBEINGUSED) return;
            HARDWAREMAP_NAME = bot.USE_LOGITECH_WEBCAM ? HARDWAREMAP_NAME : "";
            ISBEINGUSED = bot.USE_LOGITECH_WEBCAM;
        }

    }
    public LOGITECH_WEBCAM_PARAMS params;
    private final boolean isBeingUsed;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    /** The raw FTC AprilTag pose of the most-recent detection. */
    public AprilTagPoseFtc AprilTagPose;
    /** Full 3D bot pose from the most-recent detection. */
    public Pose3D botpose;
    /** Pedro-coordinate 2D pose (x, z → x, y; yaw → heading). */
    public com.pedropathing.geometry.Pose pedroPose;
    /** The tag ID of the most-recent detection. */
    public int id;
    /** True when at least one tag was detected in the latest {@link #update()} call. */
    public boolean hasDetection;
    /** True when the camera is disconnected or unavailable. */
    public boolean isDisconnected;

    /** All current detections from the last {@link #update()} call. */
    private List<AprilTagDetection> currentDetections;

    // ==================== CONSTRUCTORS ====================

    /** Constructs with a specific camera hardware name. */
    public LogitechWebcam(HardwareMap hwMap, LOGITECH_WEBCAM_PARAMS params, RobotConstants bot) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hwMap.get(WebcamName.class, params.HARDWAREMAP_NAME))
                .setCameraResolution(new android.util.Size(640, 480))
                .build();
        isDisconnected = false;
        this.params = params;
        this.params.updateFromBot(bot);
        isBeingUsed = params.ISBEINGUSED;
    }

    /** No-op constructor when the camera is not being used. */
    public LogitechWebcam() {
        isBeingUsed = false;
        isDisconnected = true;
    }

    // ==================== UPDATE ====================

    /**
     * Polls the VisionPortal for new AprilTag detections.
     * If at least one tag is found, the first detection's pose is stored
     * in {@link #AprilTagPose}, {@link #botpose}, and {@link #pedroPose}.
     */
    public void update() {
        if (!isBeingUsed) return;

        try {
            currentDetections = tagProcessor.getDetections();

            if (currentDetections == null || currentDetections.isEmpty()) {
                hasDetection = false;
                return;
            }

            // Use the first (highest-confidence) detection
            AprilTagDetection tag = currentDetections.get(0);
            if (tag.ftcPose == null) {
                hasDetection = false;
                return;
            }

            AprilTagPose = tag.ftcPose;
            id = tag.id;

            Position position = new Position(params.OUTPUT_DISTANCE_UNIT,
                    AprilTagPose.x, AprilTagPose.y, AprilTagPose.z, 0);
            YawPitchRollAngles orientation = new YawPitchRollAngles(
                    params.OUTPUT_ANGLE_UNIT,
                    AprilTagPose.yaw, AprilTagPose.pitch, AprilTagPose.roll, 0);
            botpose = new Pose3D(position, orientation);
            pedroPose = toPedro(botpose);

            hasDetection = true;
            isDisconnected = false;
        } catch (Exception e) {
            hasDetection = false;
            isDisconnected = true;
        }
    }

    // ==================== QUERIES ====================

    /**
     * Gets a detection result for a specific tag ID, or null if not visible.
     * @param tagID The AprilTag fiducial ID
     * @return The detection, or null
     */
    public AprilTagDetection getDetectionForTag(int tagID) {
        if (!isBeingUsed || currentDetections == null) return null;
        for (AprilTagDetection d : currentDetections) {
            if (d.id == tagID) return d;
        }
        return null;
    }

    /**
     * @return true if the specified tag is currently visible
     */
    public boolean isTagVisible(int tagID) {
        return getDetectionForTag(tagID) != null;
    }

    /**
     * @return All current AprilTag detections, or null if camera is not in use
     */
    public List<AprilTagDetection> getDetections() {
        if (!isBeingUsed) return null;
        return currentDetections;
    }

    /**
     * @return The number of tags currently detected
     */
    public int getDetectionCount() {
        if (currentDetections == null) return 0;
        return currentDetections.size();
    }

    // ==================== HELPERS ====================

    /**
     * Converts a Pose3D to a Pedro Pathing Pose (field-relative 2D).
     * Uses x and z from the 3D position (camera forward = z) and yaw for heading.
     */
    private com.pedropathing.geometry.Pose toPedro(Pose3D pose3D) {
        return new com.pedropathing.geometry.Pose(
                pose3D.getPosition().toUnit(DistanceUnit.INCH).x,
                pose3D.getPosition().toUnit(DistanceUnit.INCH).z,
                pose3D.getOrientation().getYaw(AngleUnit.RADIANS)
        );
    }
}
