package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.LogitechWebcam;

@TeleOp(name = "FTC SDK Webcam Test", group = "1: Tests")
public class WebcamFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (skipIfMechanismMissing(LogitechWebcam.class, telemetry)) {
            return;
        }

        LogitechWebcam webcam = Constants.createWebcam(hardwareMap);
        showStep(telemetry, "Polling AprilTag webcam wrapper");
        for (int i = 0; i < 8; i++) {
            webcam.update();
            sleep(100);
        }

        telemetry.addData("Disconnected", webcam.isDisconnected);
        telemetry.addData("Has Detection", webcam.hasDetection);
        telemetry.addData("Detection Count", webcam.getDetectionCount());
        telemetry.addData("Last Tag ID", webcam.hasDetection ? webcam.id : -1);
        telemetry.addLine("Point the webcam at a visible AprilTag before pressing Start.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Webcam Factory Test";
    }
}
