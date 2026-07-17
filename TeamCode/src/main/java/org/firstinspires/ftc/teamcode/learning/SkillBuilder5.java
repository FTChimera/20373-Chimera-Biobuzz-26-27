package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SkillBuilder5 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FRM = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor FLM = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor BRM = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor BLM = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor Intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor Outtake = hardwareMap.dcMotor.get("outtakeMotor");
        DcMotor Transfer = hardwareMap.dcMotor.get("transferMotor");

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //takes position front or back position of joystick, and reverse. Now front gives  positive (vice versa)
            double x = gamepad1.left_stick_x * 1.1; // * 1.1 makes strafing perfect
            double right = gamepad1.right_stick_x; //right stick left/right

            if (gamepad1.right_trigger_pressed) {
                Intake.setPower(1.0);
            } else {
                Intake.setPower(0.0);
            }

            if (gamepad1.left_trigger_pressed) {
                Outtake.setPower(1.0);
                if (Intake.getPower() == 1.0) {
                    Transfer.setPower(1.0);
                }
            } else {
                Outtake.setPower(0.0);
                Transfer.setPower(0.0);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(right), 1);

            double frontLeftPower = (y + x + right) / denominator;
            double backLeftPower = (y - x + right) / denominator;
            double frontRightPower = (y - x - right) / denominator;
            double backRightPower = (y + x - right) / denominator;
            FLM.setPower(frontLeftPower);
            BLM.setPower(backLeftPower);
            FRM.setPower(frontRightPower);
            BRM.setPower(backRightPower);

        }


    }

}
