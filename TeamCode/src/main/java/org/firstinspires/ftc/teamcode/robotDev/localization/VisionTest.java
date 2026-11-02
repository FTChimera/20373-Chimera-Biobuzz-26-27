package org.firstinspires.ftc.teamcode.robotDev.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.concurrent.TimeUnit;

@Config
@Utility(name = "Vision Test", description = "Limelight diagnostics, transform calibration, and relocalization checks")
public class VisionTest extends LinearOpMode {
    public static boolean RED_ALLIANCE = true;
    public static double DRIVE_SCALE = 0.6;
    public static boolean HOLD_STILL = false;

    @Override
    public void runOpMode() {
        AllianceColor allianceColor = RED_ALLIANCE ? AllianceColor.RED : AllianceColor.BLUE;
        CommonKt.init(allianceColor);
        Constants.PINPOINT_CONFIG.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.localizer.setPose(StartPoses.gardenArea);
        Vision vision = new Vision(hardwareMap, allianceColor);
        Field field = new Field(allianceColor, 13.835, 17.008);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Alliance", allianceColor);
        telemetry.addLine("A: re-zero localizer from vision (when reliability is OK)");
        telemetry.addLine("Drive robot while tuning Vision.FIELD_* offsets in FTC Dashboard");
        telemetry.update();

        waitForStart();
        vision.start();
        Timer timer = new Timer(TimeUnit.SECONDS);

        while (opModeIsActive()) {
            double dt = timer.dt(TimeUnit.SECONDS);
            if (HOLD_STILL) {
                follower.hold(follower.localizer.pose(), true);
            } else {
                follower.manual(
                        -gamepad1.left_stick_y * DRIVE_SCALE,
                        -gamepad1.left_stick_x * DRIVE_SCALE,
                        -gamepad1.right_stick_x * DRIVE_SCALE,
                        true
                );
            }
            follower.update(dt);

            vision.update(follower.localizer.pose().heading());
            vision.applyInferredHiveState(field);

            if (gamepad1.a && vision.hasReliableLocalization()) {
                follower.localizer.setPose(vision.getFieldPoseEstimate());
            }

            Pose2d fieldPose = vision.getFieldPoseEstimate();
            telemetry.addData("Follower pose", follower.localizer.pose());
            telemetry.addData("Vision field pose", fieldPose);
            telemetry.addData("Vision raw pose", vision.camTransform.getRawPose());
            telemetry.addData("Vision reliability", vision.getLocalizationReliabilityReason());
            telemetry.addData("Tag count", vision.camTransform.getTagCount());
            telemetry.addData("Pose ambiguity (m)", vision.camTransform.getAmbiguity());
            telemetry.addData("Pipeline (active/desired)", vision.getActivePipelineIndex() + "/" + vision.getDesiredPipelineIndex());
            telemetry.addData("Limelight connected", vision.limelight3A.isConnected());
            telemetry.addData("Inferred hive state", vision.getInferredHiveState());
            telemetry.addData("Field hive state", field.getHiveState());
            telemetry.addData("Hive tip classification", vision.camTransform.getHiveState());
            telemetry.update();
        }

        vision.stop();
    }
}
