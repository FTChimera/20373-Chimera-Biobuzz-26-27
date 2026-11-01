package org.firstinspires.ftc.teamcode.robotDev.localization;

import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.AUDIENCE_DOWN;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.AUDIENCE_UP;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.UNKNOWN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.localization.CamTransform;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

@Utility
public class CamRelocalizer extends LinearOpMode {
    Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, AllianceColor.RED);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        vision.start();

        while (opModeIsActive()) {
            vision.update();
            Pose2d estimatedPose = vision.getFieldPoseEstimate();
            int hiveStatePrediction = vision.camTransform.getHiveState();
            telemetry.addData("Estimated Pose", estimatedPose);
            telemetry.addData("Localization Reliability", vision.getLocalizationReliabilityReason());
            telemetry.addData("Tag Count", vision.camTransform.getTagCount());
            telemetry.addData("Hive State", hiveState(hiveStatePrediction));
            telemetry.addData("HIVE ANGLE", vision.camTransform.getHiveAngle());
            telemetry.addData("Is scoreable", CamTransform.isScoreable(hiveStatePrediction));
            telemetry.addData("Inferred field hive state", vision.getInferredHiveState());
            telemetry.update();
        }
        vision.stop();
    }
    String hiveState(int prediction) {
        if (prediction == MIDDLE) return "Middle";
        if (prediction == AUDIENCE_UP) return "Audience Up";
        if (prediction == AUDIENCE_DOWN) return "Audience Down";
        if (prediction == UNKNOWN) return "Unknown";
        return "Invalid Prediction";

    }
}
