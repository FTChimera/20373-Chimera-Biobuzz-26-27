package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import com.rmsftc.library.hardware.leds.RGBIndicator.RGBIndicator_PARAMS;
import com.rmsftc.library.hardware.hardwareDevice.RGBIndicator;
import com.rmsftc.library.robot.RobotConstants;

//@Disabled
@Utility
public class RGBIndicatorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        RGBIndicator_PARAMS params = new RGBIndicator_PARAMS();
        RGBIndicator rgbIndicator = hardwareMap.get(RGBIndicator.class, params.HARDWAREMAP_NAME);
        rgbIndicator.setColor(RGBIndicator.Color.RED);
        // should show red
        sleep(2000);
        telemetry.addData("Color (should be RED)", rgbIndicator.getColor());
        telemetry.update();
        sleep(2000);
        rgbIndicator.close();
        com.rmsftc.library.hardware.leds.RGBIndicator rgb2 = new com.rmsftc.library.hardware.leds.RGBIndicator(hardwareMap, new RobotConstants().addMechanism(com.rmsftc.library.hardware.leds.RGBIndicator.class));
        rgb2.setColor(com.rmsftc.library.hardware.leds.RGBIndicator.Color.BLUE);
        // should show blue
        telemetry.addData("Color2 (should be BLUE)", rgb2.getColor());
        telemetry.update();
        sleep(2000);
    }

    @Override
    protected String getTestName() {
        return "RGB Indicator Test";
    }
}
