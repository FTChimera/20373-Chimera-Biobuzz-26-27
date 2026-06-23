package org.firstinspires.ftc.teamcode.tests.ftcSdk;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.SlidesMotor;
//@Disabled
@TeleOp
public class SlidesMotorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        SlidesMotor slides = hardwareMap.get(SlidesMotor.class, "slidesMotor");
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
