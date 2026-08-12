package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

//@Disabled
@Utility
public class MotorTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        Motor motor = new Motor(hardwareMap, Constants.currentBot.FRONT_LEFT_MOTOR_NAME);
        motor.setPower(0.6);
        sleep(2000);
        motor.stopPower();
        motor.setDirection(Direction.REVERSE);
        motor.setPower(0.6);
        telemetry.addData("Motor Target Power (should be 0.6)", motor.getTargetPower());
        telemetry.update();
        sleep(2000);
        motor.stopPower();
        telemetry.update();
    }



    @Override
    protected String getTestName() {
        return "Motor Test";
    }
}
