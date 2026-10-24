package org.firstinspires.ftc.teamcode.robotDev.lights;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.hardware.lights.rev.FtcBlinkinLedDriver;
@Utility
public class CustomWrapperLEDTest extends LinearOpMode {

    FtcBlinkinLedDriver blinkin;


    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(FtcBlinkinLedDriver.class, "ledStrip");
        blinkin.setPattern(FtcBlinkinLedDriver.Pattern.BLUE);
        blinkin.setColor1(Color.BLUE);

        waitForStart();
        blinkin.setPattern(FtcBlinkinLedDriver.DEFAULT_RESTORE_PATTERN);
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                blinkin.queueSetColor1(Color.RED);
                blinkin.queueSetPattern(FtcBlinkinLedDriver.Pattern.COLOR1_SHOT);
            }
            if (gamepad1.aWasReleased()) {
                blinkin.queueSetColor1(Color.BLUE);
                blinkin.queueSetPattern(FtcBlinkinLedDriver.Pattern.COLOR1_SHOT);
            }

            if (gamepad1.bWasPressed() && !blinkin.isCommandQueueBusy()) {
                blinkin.queueSetMode(FtcBlinkinLedDriver.StripMode.MODE_12V);
            }
            if (gamepad1.bWasReleased() && !blinkin.isCommandQueueBusy()) {
                blinkin.queueSetMode(FtcBlinkinLedDriver.StripMode.MODE_5V);
            }

            if (gamepad1.back) {
                blinkin.queueSetPattern(FtcBlinkinLedDriver.Pattern.RAINBOW_RGB);
            }
        }


        telemetry.addData("Pattern", blinkin.getLastPattern());
        telemetry.addData("is command queue busy?", blinkin.isCommandQueueBusy());
        telemetry.update();
    }
}
