package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.hardware.gamepad.Button;
import com.rmsftc.library.hardware.gamepad.GamepadEx;

@Utility(name = "FIELD CENTRIC MECANUM TELEOP")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Constants.currentBot.FRONT_LEFT_MOTOR_NAME);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Constants.currentBot.BACK_LEFT_MOTOR_NAME);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Constants.currentBot.FRONT_RIGHT_MOTOR_NAME);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Constants.currentBot.BACK_RIGHT_MOTOR_NAME);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = Constants.createIMU(hardwareMap);
        imu.resetYaw();
        GamepadEx driver = new GamepadEx(gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driver.update();
            double y = driver.leftStickY() * Constants.currentBot.MAX_POWER;
            double x = driver.leftStickX() * Constants.currentBot.MAX_POWER;
            double rx = driver.rightStickX() * Constants.currentBot.MAX_POWER * 0.8;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (driver.isPressed(Button.OPTIONS)) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}