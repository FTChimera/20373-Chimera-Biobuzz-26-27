package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator.RGBIndicator_PARAMS;
import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.RGBIndicator;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

//@Disabled
@TeleOp
public class RGBIndicatorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        RGBIndicator_PARAMS params = new RGBIndicator_PARAMS();
        RGBIndicator rgbIndicator = hardwareMap.get(RGBIndicator.class, params.HARDWAREMAP_NAME);
        rgbIndicator.setColor(RGBIndicator.Color.RED);
        // should show red
        sleep(2000);
        telemetry.addData("Color (should be RED)", rgbIndicator.getColor());
        telemetry.update();
        sleep(2000);
        rgbIndicator.close();
        org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator rgb2 = new org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator(hardwareMap, new RobotConstants().addMechanism(org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator.class));
        rgb2.params.type = org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator.RGBIndicator_PARAMS.Type.ONLY_HARDWARE; // for now
        // test panels later
        rgb2.setColor(org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator.Color.BLUE);
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
