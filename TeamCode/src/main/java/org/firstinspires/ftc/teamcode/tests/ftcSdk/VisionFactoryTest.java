package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@Utility(name = "FTC SDK Vision Test")
public class VisionFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        Vision vision = Constants.createVision(hardwareMap);
        showStep(telemetry, "Starting Vision on AprilTag pipeline");
        vision.start(Vision.Pipeline.APRIL_TAG);
        for (int i = 0; i < 8; i++) {
            vision.update();
            sleep(100);
        }
        vision.stop();

        telemetry.addData("Disconnected", vision.isDisconnected());
        telemetry.addData("Last Tag ID", vision.getLastTagId());
        telemetry.addData("Approx Distance", vision.getLastDistanceInches());
        telemetry.addData("Fiducials Seen", vision.getFiducials() == null ? 0 : vision.getFiducials().size());
        telemetry.addLine("Point the camera at a visible AprilTag before pressing Start.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Vision Factory Test";
    }
}
