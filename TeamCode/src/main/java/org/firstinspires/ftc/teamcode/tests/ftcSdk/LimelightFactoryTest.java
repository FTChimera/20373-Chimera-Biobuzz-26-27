package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import com.rmsftc.library.hardware.webcam.Limelight;

@Utility(name = "FTC SDK Limelight Test")
public class LimelightFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        if (skipIfMechanismMissing(Limelight.class, telemetry)) {
            return;
        }

        Limelight limelight = Constants.createLimelight(hardwareMap);
        showStep(telemetry, "Starting Limelight on AprilTag pipeline");
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        for (int i = 0; i < 8; i++) {
            limelight.LLUpdate();
            sleep(100);
        }
        limelight.stop();

        telemetry.addData("Disconnected", limelight.isDisconnected);
        telemetry.addData("Last Tag ID", limelight.tid);
        telemetry.addData("Approx Distance", limelight.dist);
        telemetry.addData("Fiducials Seen", limelight.getFiducials() == null ? 0 : limelight.getFiducials().size());
        telemetry.addLine("Point the camera at a visible AprilTag before pressing Start.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Limelight Factory Test";
    }
}
