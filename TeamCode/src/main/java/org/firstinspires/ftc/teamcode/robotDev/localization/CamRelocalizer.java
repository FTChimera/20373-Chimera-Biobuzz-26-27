package org.firstinspires.ftc.teamcode.robotDev.localization;

import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.AUDIENCE_DOWN;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.AUDIENCE_UP;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.localization.CamTransform.UNKNOWN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.localization.CamTransform;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

@Utility
public class CamRelocalizer extends LinearOpMode {
    CamTransform camTransform;


    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        camTransform = new CamTransform(limelight3A, AllianceColor.RED);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        limelight3A.start();

        while (opModeIsActive()) {
            camTransform.update();
            Pose2d estimatedPose = camTransform.getPose();
            int hiveStatePrediction = camTransform.getHiveState();
            telemetry.addData("Estimated Pose", estimatedPose);
            telemetry.addData("Hive State", hiveState(hiveStatePrediction));
            telemetry.addData("HIV ANGLE", camTransform.getHiveAngle());
            telemetry.addData("Is scoreable", CamTransform.isScoreable(hiveStatePrediction));
            telemetry.addData("Is valid", CamTransform.isValid(hiveStatePrediction));
            telemetry.update();
        }
    }
    String hiveState(int prediction) {
        if (prediction == MIDDLE) return "Middle";
        if (prediction == AUDIENCE_UP) return "Audience Up";
        if (prediction == AUDIENCE_DOWN) return "Audience Down";
        if (prediction == UNKNOWN) return "Unknown";
        return "Invalid Prediction";

    }
}
