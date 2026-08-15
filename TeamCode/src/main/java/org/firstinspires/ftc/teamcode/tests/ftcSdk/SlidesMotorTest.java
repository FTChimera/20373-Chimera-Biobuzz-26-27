package org.firstinspires.ftc.teamcode.tests.ftcSdk;


import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import com.rmsftc.library.hardware.hardwareDevice.SlidesMotor;
//@Disabled
@Utility
public class SlidesMotorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        SlidesMotor slides = hardwareMap.get(SlidesMotor.class, SlidesMotor.DEFAULT_HARDWAREMAP_NAME);
        slides.initialize(12, 1355);
        sleep(200);
        slides.setUp();
        sleep(2000);
        slides.setDown();
        sleep(2000);
        slides.setPositionPercentage(0.5);
        sleep(2000);
        telemetry.addData("Slides Position (should be 0.5)", slides.getPositionPercentage());
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "Slides Test";
    }
}
