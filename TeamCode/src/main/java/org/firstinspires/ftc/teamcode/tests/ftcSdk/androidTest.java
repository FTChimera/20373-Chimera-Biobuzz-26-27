package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
// ADD @Disabled for all tests once they are run.

public abstract class androidTest extends LinearOpMode {
    protected abstract void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry);
    // Leave out blackboard because we already have our own

    protected abstract String getTestName();

    @Override
    public void runOpMode() {
        telemetry.addData("Test Name", getTestName());
        waitForStart();
        test(hardwareMap, gamepad1, gamepad2, telemetry);
        sleep(2000); // Sleep for 2 seconds then stop opmode
        requestOpModeStop();
    }
}
