package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

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
        GamepadEx driver = new GamepadEx(gamepad1);

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            driver.update();
            double y = driver.leftStickY();
            double x = driver.leftStickX() * 1.1; // * 1.1 makes strafing perfect
            double right = driver.rightStickX(); //right stick left/right

            if (driver.rightTrigger.isPressed()) {
                Intake.setPower(1.0);
            } else {
                Intake.setPower(0.0);
            }

            if (driver.leftTrigger.isPressed()) {
                Outtake.setPower(1.0);
                if (Outtake.getPower() == 1.0) {
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
