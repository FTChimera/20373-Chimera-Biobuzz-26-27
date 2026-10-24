package org.firstinspires.ftc.teamcode.robotDev.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Utility
public class VisionRelocalizer extends LinearOpMode {
    Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, AllianceColor.RED);

        waitForStart();
        vision.start();
        while (opModeIsActive()) {
            vision.update();
            LLResult result = vision.getLatestResult(); 
        }
    }

}
