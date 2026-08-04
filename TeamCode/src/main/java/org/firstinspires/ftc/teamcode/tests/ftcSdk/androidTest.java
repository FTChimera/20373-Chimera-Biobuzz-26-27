package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.DualGamepad;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.HashMap;
// ADD @Disabled for all tests once they are run.

public abstract class androidTest extends LinearOpMode {
    protected abstract void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry);
    // Leave out blackboard because we already have our own

    protected abstract String getTestName();

    @Override
    public void runOpMode() {
        DualGamepad gamepads = new DualGamepad(gamepad1, gamepad2);
        telemetry.addData("Test Name", getTestName());
        telemetry.update();
        waitForStart();
        gamepads.update();
        test(hardwareMap, gamepads.gamepad1, gamepads.gamepad2, telemetry);
        sleep(2000); // Sleep for 2 seconds then stop opmode
        requestOpModeStop();
    }

    protected final boolean skipIfMechanismMissing(Class<? extends Mechanism> mechanismClass, Telemetry telemetry) {
        if (Constants.hasMechanism(mechanismClass)) {
            return false;
        }
        telemetry.addData("Skipped", "%s is not configured in Constants.currentBot", mechanismClass.getSimpleName());
        telemetry.addData("Configured Mechanisms", Constants.getMechanisms());
        telemetry.update();
        return true;
    }

    protected final void showStep(Telemetry telemetry, String step) {
        telemetry.addData("Step", step);
        telemetry.update();
    }
}
