package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Utility(name = "FTC SDK Intake Test")
public class IntakeFactoryTest extends androidTest {
    private static final double SAFE_POWER = 0.25;

    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        if (skipIfMechanismMissing(Intake.class, telemetry)) {
            return;
        }

        Intake intake = Constants.createIntake(hardwareMap);
        showStep(telemetry, "Running intake forward at 25% power");
        intake.setPower(SAFE_POWER);
        sleep(700);

        showStep(telemetry, "Running intake reverse at 25% power");
        intake.setPower(-SAFE_POWER);
        sleep(700);

        intake.Stop();
        telemetry.addData("Target Power After Stop", intake.getTargetPower());
        telemetry.addLine("Keep hands and game elements clear before pressing Start.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Intake Factory Test";
    }
}
