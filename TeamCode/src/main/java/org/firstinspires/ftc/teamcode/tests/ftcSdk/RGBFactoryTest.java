package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import com.rmsftc.library.hardware.leds.RGBIndicator;

@Utility(name = "FTC SDK RGB Factory Test")
public class RGBFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        if (skipIfMechanismMissing(RGBIndicator.class, telemetry)) {
            return;
        }

        RGBIndicator indicator = Constants.createRGB(hardwareMap);
        showStep(telemetry, "Setting RGB wrapper to BLUE");
        indicator.setColor(RGBIndicator.Color.BLUE);
        sleep(700);
        showStep(telemetry, "Setting RGB wrapper to GREEN");
        indicator.setColor(RGBIndicator.Color.GREEN);
        sleep(700);
        indicator.setColor(RGBIndicator.Color.BLACK);

        telemetry.addData("Reported Color", indicator.getColor());
        telemetry.addLine("This exercises the profile-managed RGB wrapper path.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK RGB Factory Test";
    }
}
