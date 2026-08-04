package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.Slides1;

@TeleOp(name = "FTC SDK Slides1 Test", group = "1: Tests")
public class Slides1FactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (skipIfMechanismMissing(Slides1.class, telemetry)) {
            return;
        }

        Slides1 slides = Constants.createSlides1(hardwareMap);
        telemetry.addData("Slides1 Created", slides != null);
        telemetry.addLine("This currently validates the Slides1 factory/constructor path only.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Slides1 Factory Test";
    }
}
