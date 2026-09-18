package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.RGBIndicator;

//@Disabled
@Utility
public class RGBIndicatorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        RGBIndicator rgbIndicator = hardwareMap.get(
                RGBIndicator.class, RGBIndicator.DEFAULT_HARDWAREMAP_NAME);
        rgbIndicator.setColor(RGBIndicator.Color.RED);
        // should show red
        sleep(2000);
        telemetry.addData("Color (should be RED)", rgbIndicator.getColor());
        telemetry.update();
        sleep(2000);
        rgbIndicator.close();
    }

    @Override
    protected String getTestName() {
        return "RGB Indicator Test";
    }
}
