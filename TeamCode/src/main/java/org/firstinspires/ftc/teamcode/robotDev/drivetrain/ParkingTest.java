package org.firstinspires.ftc.teamcode.robotDev.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Config
@Utility(name = "Parking Test", description = "Manual drive with on-demand moveToParkZone diagnostics")
public class ParkingTest extends LinearOpMode {
    public enum StartPosePreset {
        GARDEN,
        LOADING
    }

    private enum ParkState {
        MANUAL,
        FOLLOWING,
        ARRIVED,
        FAILED
    }

    public static boolean USE_OPMODE_STORAGE = true;
    public static boolean RED_ALLIANCE = false;
    public static StartPosePreset START_POSE_PRESET = StartPosePreset.GARDEN;
    public static boolean USE_CUSTOM_START_POSE = false;
    public static double CUSTOM_START_X_IN = 9.0;
    public static double CUSTOM_START_Y_IN = 25.0;
    public static double CUSTOM_START_HEADING_DEG = 0.0;

    private Robot robot;
    private AllianceColor selectedAlliance;
    private Pose2d selectedStartPose;

    private GpFancyButton requestPark;
    private GpFancyButton retryPark;
    private GpFancyButton cancelPark;

    private ParkState parkState = ParkState.MANUAL;
    private String parkStatusReason = "Waiting for park request.";
    private boolean parkPathStarted;
    private boolean parkArrived;
    private boolean lastParkStartSucceeded;
    private int parkAttempts;
    private double parkElapsedAtArrivalSec = Double.NaN;

    private boolean defaultFollowerHoldEnd;
    private boolean defaultFollowerBrakeAtEnd;

    private Timer parkTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        setupRobot();
        setupControls();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Parking Test ready");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Start Pose", selectedStartPose);
        telemetry.addLine("Controls: LS=drive, RSX=turn, A=request park, X=retry failed park, B=cancel park");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            cleanupRobot();
            return;
        }

        robot.start();

        try {
            while (opModeIsActive()) {
                requestPark.update();
                retryPark.update();
                cancelPark.update();

                if (cancelPark.isPressed()) {
                    cancelParking("Driver canceled park; manual control restored.");
                }

                if (requestPark.isPressed() && parkState == ParkState.MANUAL) {
                    requestParking("A button request");
                }

                if (retryPark.isPressed() && parkState == ParkState.FAILED) {
                    requestParking("X button retry");
                }

                if (isManualDriveEnabled()) {
                    robot.setTeleOpDrive(
                            new DriveVector(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            ),
                            true,
                            false
                    );
                }

                robot.update();
                evaluateParkingProgress();
                sendTelemetry();
            }
        } finally {
            cleanupRobot();
        }
    }

    private void setupRobot() {
        if (!OpModeStorage.initialized) {
            OpModeStorage.resetToDefault();
        }

        selectedAlliance = USE_OPMODE_STORAGE
                ? OpModeStorage.allianceColor
                : AllianceColor.Companion.from(RED_ALLIANCE);

        CommonKt.init(selectedAlliance);
        selectedStartPose = resolveStartPose();

        robot = new Robot(telemetry, hardwareMap, selectedAlliance, RobotMode.TEST, selectedStartPose);
        robot.debug = false;
        robot.setManualBrake(false);
        robot.setSmartIntakeControl(false);
        robot.setSmartShooterControl(false);

        defaultFollowerHoldEnd = robot.follower.holdEnd.get();
        defaultFollowerBrakeAtEnd = Constants.FOLLOWER_BRAKE_AT_END.get();

        parkTimer = new Timer(TimeUnit.SECONDS);
        parkPathStarted = false;
        parkArrived = false;
        parkAttempts = 0;
        lastParkStartSucceeded = false;
        parkElapsedAtArrivalSec = Double.NaN;
    }

    private void setupControls() {
        requestPark = new GpFancyButton(FancyButton.PressType.RisingEdge, () -> gamepad1.a);
        retryPark = new GpFancyButton(FancyButton.PressType.RisingEdge, () -> gamepad1.x);
        cancelPark = new GpFancyButton(FancyButton.PressType.RisingEdge, () -> gamepad1.b);
    }

    private Pose2d resolveStartPose() {
        if (USE_OPMODE_STORAGE && OpModeStorage.pose != null) {
            return OpModeStorage.pose;
        }

        if (USE_CUSTOM_START_POSE) {
            return PoseFactory.deg(CUSTOM_START_X_IN, CUSTOM_START_Y_IN, CUSTOM_START_HEADING_DEG);
        }

        if (START_POSE_PRESET == StartPosePreset.LOADING) {
            return StartPoses.loadingArea;
        }

        return StartPoses.gardenArea;
    }

    private void requestParking(String source) {
        parkAttempts++;
        lastParkStartSucceeded = robot.moveToParkZone();

        if (!lastParkStartSucceeded) {
            parkState = ParkState.FAILED;
            parkPathStarted = false;
            parkArrived = false;
            parkElapsedAtArrivalSec = Double.NaN;
            parkStatusReason = "Park start failed (no route). Press X to retry.";
            restoreFollowerParkSettings();
            return;
        }

        parkPathStarted = robot.follower.following();
        if (parkPathStarted) {
            parkTimer.reset();
            parkElapsedAtArrivalSec = Double.NaN;
            parkArrived = false;
            parkState = ParkState.FOLLOWING;
            parkStatusReason = source + " succeeded; following path to loading zone.";
            return;
        }

        Pose2d correctedPose = robot.getPoseCompensatedForVelocity();
        parkArrived = robot.field.isRobotOverlappingLoadingZone(correctedPose);
        if (parkArrived) {
            robot.breakFollowing();
            parkState = ParkState.ARRIVED;
            parkElapsedAtArrivalSec = 0.0;
            parkStatusReason = "Already overlapping loading zone; no path required.";
        } else {
            parkState = ParkState.FAILED;
            parkElapsedAtArrivalSec = Double.NaN;
            parkStatusReason = "Park request ended without active follower path. Press X to retry.";
            restoreFollowerParkSettings();
        }
    }

    private void evaluateParkingProgress() {
        if (parkState != ParkState.FOLLOWING || !parkPathStarted) {
            return;
        }

        Pose2d compensatedPose = robot.getPoseCompensatedForVelocity();
        boolean overlap = robot.field.isRobotOverlappingLoadingZone(compensatedPose);
        boolean pathComplete = !robot.follower.following() || (robot.follower.atParametricEnd() && !robot.follower.isBusy());

        if (overlap && pathComplete) {
            parkArrived = true;
            parkState = ParkState.ARRIVED;
            parkElapsedAtArrivalSec = parkTimer.get(TimeUnit.SECONDS);
            parkStatusReason = "Path completed and loading-zone overlap confirmed.";
            return;
        }

        if (pathComplete && !overlap) {
            parkState = ParkState.FAILED;
            parkPathStarted = false;
            parkArrived = false;
            parkElapsedAtArrivalSec = Double.NaN;
            parkStatusReason = "Follower settled outside loading zone. Press X to retry.";
            robot.breakFollowing();
            restoreFollowerParkSettings();
        }
    }

    private void cancelParking(String reason) {
        robot.breakFollowing();
        restoreFollowerParkSettings();
        parkPathStarted = false;
        parkArrived = false;
        parkElapsedAtArrivalSec = Double.NaN;
        parkState = ParkState.MANUAL;
        parkStatusReason = reason;
    }

    private boolean isManualDriveEnabled() {
        return parkState == ParkState.MANUAL || parkState == ParkState.FAILED;
    }

    private void restoreFollowerParkSettings() {
        robot.follower.holdEnd.set(defaultFollowerHoldEnd);
        Constants.FOLLOWER_BRAKE_AT_END.set(defaultFollowerBrakeAtEnd);
    }

    private void cleanupRobot() {
        if (robot == null) {
            return;
        }
        restoreFollowerParkSettings();
        robot.stopEverything();
    }

    private void sendTelemetry() {
        Pose2d pose = robot.getPose();
        Pose2d compensatedPose = robot.getPoseCompensatedForVelocity();
        double distanceRaw = robot.field.getClosestLoadingZoneDistanceToRobot(pose);
        double distanceCompensated = robot.distanceToLoadingZone();
        boolean overlap = robot.field.isRobotOverlappingLoadingZone(compensatedPose);

        telemetry.addLine("=== PARKING TEST ===");
        telemetry.addLine("Controls: LS=drive, RSX=turn, A=request park, X=retry after failure, B=cancel to manual");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Configured start pose", selectedStartPose);
        telemetry.addData("Manual control enabled", isManualDriveEnabled());

        telemetry.addData("Pose", pose);
        telemetry.addData("Compensated pose", compensatedPose);
        telemetry.addData("Distance to loading zone (raw in)", String.format(Locale.US, "%.2f", distanceRaw));
        telemetry.addData("Distance to loading zone (comp in)", String.format(Locale.US, "%.2f", distanceCompensated));
        telemetry.addData("isRobotOverlappingLoadingZone", overlap);

        telemetry.addData("Park state", parkState);
        telemetry.addData("Park status", parkStatusReason);
        telemetry.addData("Last park start success", lastParkStartSucceeded);
        telemetry.addData("Park path started", parkPathStarted);
        telemetry.addData("Park arrived", parkArrived);
        telemetry.addData("Park attempts", parkAttempts);
        telemetry.addData(
                "Park elapsed (s)",
                parkPathStarted
                        ? String.format(Locale.US, "%.2f", Double.isNaN(parkElapsedAtArrivalSec) ? parkTimer.get(TimeUnit.SECONDS) : parkElapsedAtArrivalSec)
                        : "N/A"
        );

        telemetry.addData("Follower mode", robot.follower.mode());
        telemetry.addData("Follower following", robot.follower.following());
        telemetry.addData("Follower holding", robot.follower.holding());
        telemetry.addData("Follower busy", robot.follower.isBusy());
        telemetry.addData("Follower at parametric end", robot.follower.atParametricEnd());
        telemetry.addData("Follower completion", String.format(Locale.US, "%.2f", robot.follower.completion()));
        telemetry.addData("Follower remaining distance", String.format(Locale.US, "%.2f", robot.follower.remainingDistance()));
        telemetry.addData("Follower distance to endpoint", String.format(Locale.US, "%.2f", robot.follower.distanceToEndpoint()));
        telemetry.addData("Follower path index", robot.follower.pathIndex());
        telemetry.addData("Follower holdEnd", robot.follower.holdEnd.get());
        telemetry.addData("Follower brakeAtEnd", Constants.FOLLOWER_BRAKE_AT_END.get());

        telemetry.update();
    }
}
