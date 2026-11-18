package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.pedro.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.UtilityPoses;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

import java.util.ArrayList;
import java.util.Locale;

@Utility
public class PoseCollector extends LinearOpMode {
    Follower follower;
    ArrayList<Pose> poses = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.localizer.setPose(UtilityPoses.cornerGardenArea);
        telemetry.addLine("Start at Corner Garden Area");
        waitForStart();
        while (opModeIsActive()) {
            follower.manual(ManualDrive.fieldCentric(new DriveVector(-gamepad1.left_stick_y*0.4, -gamepad1.left_stick_x*0.4, -gamepad1.right_stick_x*0.4), follower.localizer.pose().heading()));
            follower.update();
            telemetry.addData("Pose", formatPose(follower.localizer.pose()));
            telemetry.addLine();
            telemetry.addData("Pose History", "");
            if (!poses.isEmpty()) {
                for (int i = 0; i < poses.size(); i++) {
                    telemetry.addData("Pose " + (i+1), formatPose(poses.get(i)));
                }
            } else {
                telemetry.addLine("Empty.\n Press A to add the current Pose, B to delete. Press X to relocalize at the Garden Corner.");
            }
            if (gamepad1.aWasPressed()) {
                poses.add(follower.localizer.pose());
            }
            if (gamepad1.bWasPressed()) {
                if (!poses.isEmpty()) poses.remove(poses.size()-1);
            }
            if (gamepad1.xWasPressed()) {
                follower.localizer.setPose(UtilityPoses.cornerGardenArea);
            }
            telemetry.update();
        }
    }


    public String formatPose(Pose pose) {
        return String.format(Locale.US,"x: %.2f, y: %.2f, heading: %.2f", pose.x(), pose.y(), pose.heading()/MathUtils.DEG_TO_RAD);
    }
}
