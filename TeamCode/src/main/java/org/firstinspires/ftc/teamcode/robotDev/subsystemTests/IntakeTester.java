package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@Utility
public class IntakeTester extends OpMode {
    DcMotor intakeMotor;
    CRServo left, right;
    public void init() {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        left = hardwareMap.get(CRServo.class, "servoLeft");
        right = hardwareMap.get(CRServo.class, "servoRight");
    }
    public void loop() {
        intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        left.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        right.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
